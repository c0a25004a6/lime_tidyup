#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr double SWEEP_STEP_M = 0.00001;
constexpr double TOLERANCE = 1e-12;

std::string read_file(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::vector<std::string> split(const std::string& text, char delimiter)
{
  std::vector<std::string> values;
  std::istringstream stream(text);
  std::string value;
  while (std::getline(stream, value, delimiter))
    values.push_back(value);
  return values;
}

double number(const std::string& text)
{
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
    throw std::runtime_error("invalid number: " + text);
  return value;
}

std::size_t integer(const std::string& text)
{
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size())
    throw std::runtime_error("invalid integer: " + text);
  return static_cast<std::size_t>(value);
}

std::string join_set(const std::set<std::string>& values)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto& value : values)
  {
    if (!first)
      stream << ',';
    first = false;
    stream << value;
  }
  return stream.str();
}

struct State
{
  std::string label;
  Eigen::Isometry3d world_root = Eigen::Isometry3d::Identity();
  std::vector<double> arm;
};

struct FixtureObject
{
  std::string name;
  Eigen::Vector3d size;
  Eigen::Vector3d offset_link7;
  shapes::ShapeConstPtr shape;
};

struct Fixture
{
  std::string reference_link;
  std::size_t anchor_index = 0;
  std::size_t candidate_start_index = 0;
  std::size_t state_count = 0;
  std::vector<FixtureObject> objects;
};

std::vector<State> read_states(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  std::vector<State> states;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields.size() != 16)
      throw std::runtime_error("invalid state row");
    State state;
    state.label = fields[0];
    Eigen::Quaterniond orientation(
      number(fields[7]), number(fields[4]), number(fields[5]), number(fields[6]));
    if (orientation.norm() < 1e-12)
      throw std::runtime_error("zero root quaternion");
    orientation.normalize();
    state.world_root.linear() = orientation.toRotationMatrix();
    state.world_root.translation() = Eigen::Vector3d(
      number(fields[1]), number(fields[2]), number(fields[3]));
    for (std::size_t index = 8; index <= 13; ++index)
      state.arm.push_back(number(fields[index]));
    static_cast<void>(number(fields[14]));
    static_cast<void>(number(fields[15]));
    states.push_back(std::move(state));
  }
  if (states.empty())
    throw std::runtime_error("state matrix is empty");
  return states;
}

Fixture read_fixture(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  Fixture fixture;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields[0] == "META")
    {
      if (fields.size() != 6 || fields[1] != "1")
        throw std::runtime_error("invalid fixture META");
      fixture.reference_link = fields[2];
      fixture.anchor_index = integer(fields[3]);
      fixture.candidate_start_index = integer(fields[4]);
      fixture.state_count = integer(fields[5]);
    }
    else if (fields[0] == "OBJECT")
    {
      if (fields.size() != 8)
        throw std::runtime_error("invalid fixture OBJECT");
      FixtureObject object;
      object.name = fields[1];
      object.size = Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
      object.offset_link7 = Eigen::Vector3d(number(fields[5]), number(fields[6]), number(fields[7]));
      if ((object.size.array() <= 0.0).any())
        throw std::runtime_error("nonpositive fixture size");
      object.shape.reset(new shapes::Box(object.size.x(), object.size.y(), object.size.z()));
      fixture.objects.push_back(std::move(object));
    }
    else
      throw std::runtime_error("unknown fixture row");
  }
  if (fixture.reference_link != "link7" || fixture.objects.size() != 2 ||
      fixture.objects[0].name != "rubiks_cube" || fixture.objects[1].name != "rubiks_support")
    throw std::runtime_error("fixture contract mismatch");
  return fixture;
}

std::set<std::string> contact_pairs(const collision_detection::CollisionResult& result)
{
  std::set<std::string> pairs;
  for (const auto& entry : result.contacts)
  {
    std::string first = entry.first.first;
    std::string second = entry.first.second;
    if (second < first)
      std::swap(first, second);
    pairs.insert(first + "|" + second);
  }
  return pairs;
}

collision_detection::CollisionResult check_object(
  planning_scene::PlanningScene& scene,
  const moveit::core::RobotState& state,
  const collision_detection::AllowedCollisionMatrix& acm,
  const std::string& name,
  const shapes::ShapeConstPtr& shape,
  const Eigen::Isometry3d& pose,
  const collision_detection::CollisionRequest& request)
{
  scene.getWorldNonConst()->clearObjects();
  scene.getWorldNonConst()->addToObject(name, shape, pose);
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state, acm);
  return result;
}

std::vector<double> sweep_values(double lower, double upper)
{
  if (!(lower < upper))
    throw std::runtime_error("invalid gripper bounds");
  std::vector<double> values;
  const std::size_t steps = static_cast<std::size_t>(std::floor((upper - lower) / SWEEP_STEP_M + TOLERANCE));
  values.reserve(steps + 2);
  for (std::size_t index = 0; index <= steps; ++index)
  {
    double value = upper - static_cast<double>(index) * SWEEP_STEP_M;
    if (value < lower)
      value = lower;
    values.push_back(value);
  }
  if (values.back() > lower + TOLERANCE)
    values.push_back(lower);
  else
    values.back() = lower;
  return values;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 9)
      throw std::runtime_error("usage: gripper_sweep URDF SRDF STATES FIXTURE DX DY DZ OUTPUT");

    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
      throw std::runtime_error("URDF parsing failed");
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
      throw std::runtime_error("SRDF parsing failed");
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    if (arm_group == nullptr || robot_model->getLinkModel("link7") == nullptr)
      throw std::runtime_error("required arm model missing");
    if (robot_model->getRootLinkName() != "base_footprint")
      throw std::runtime_error("root link mismatch");

    const auto states = read_states(argv[3]);
    const auto fixture = read_fixture(argv[4]);
    if (fixture.state_count != states.size() || fixture.anchor_index >= states.size())
      throw std::runtime_error("fixture/state identity mismatch");
    const Eigen::Vector3d delta(number(argv[5]), number(argv[6]), number(argv[7]));
    if ((delta - Eigen::Vector3d(0.06325, 0.0, 0.01225)).norm() > 1e-12)
      throw std::runtime_error("selected PR26 translation mismatch");

    const auto left_joint = urdf_model->getJoint("gripper_left_joint");
    const auto right_joint = urdf_model->getJoint("gripper_right_joint");
    if (!left_joint || !right_joint || !left_joint->limits || !right_joint->mimic)
      throw std::runtime_error("gripper joint contract missing");
    const double lower = left_joint->limits->lower;
    const double upper = left_joint->limits->upper;
    if (std::abs(lower + 0.01) > 1e-12 || std::abs(upper - 0.019) > 1e-12 ||
        right_joint->mimic->joint_name != "gripper_left_joint" ||
        std::abs(right_joint->mimic->multiplier - 1.0) > 1e-12)
      throw std::runtime_error("gripper limit/mimic contract mismatch");

    moveit::core::RobotState anchor(robot_model);
    anchor.setToDefaultValues();
    anchor.setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
    anchor.setVariablePosition("gripper_left_joint", upper);
    anchor.update();
    const Eigen::Isometry3d root_link7 = anchor.getGlobalLinkTransform("link7");

    std::vector<Eigen::Isometry3d> object_poses;
    for (const auto& object : fixture.objects)
    {
      Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
      link7_object.translation() = object.offset_link7 + delta;
      object_poses.push_back(root_link7 * link7_object);
    }

    const Eigen::Matrix3d root_rotation_world = states[fixture.anchor_index].world_root.linear();
    const Eigen::Vector3d normal_root = root_rotation_world.transpose() * Eigen::Vector3d::UnitZ();
    const double ground_offset_root = states[fixture.anchor_index].world_root.translation().z();
    shapes::ShapeConstPtr ground_shape(new shapes::Plane(
      normal_root.x(), normal_root.y(), normal_root.z(), ground_offset_root));

    planning_scene::PlanningScene scene(robot_model);
    const auto original_acm = scene.getAllowedCollisionMatrix();
    auto fixture_acm = original_acm;
    const auto links = robot_model->getLinkModelNames();
    for (const auto& first : links)
      for (const auto& second : links)
        fixture_acm.setEntry(first, second, true);
    for (const auto& object : fixture.objects)
      for (const auto& link : links)
        fixture_acm.setEntry(object.name, link, false);

    auto ground_acm = original_acm;
    for (const auto& first : links)
      for (const auto& second : links)
        ground_acm.setEntry(first, second, true);
    for (const auto& link : links)
      ground_acm.setEntry("ground_plane", link, true);
    ground_acm.setEntry("ground_plane", "gripper_left_link", false);
    ground_acm.setEntry("ground_plane", "gripper_right_link", false);

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.distance = false;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    const std::set<std::string> allowed_cube_pairs = {
      "gripper_left_link|rubiks_cube", "gripper_right_link|rubiks_cube"
    };
    const auto values = sweep_values(lower, upper);

    std::ofstream output(argv[8]);
    if (!output)
      throw std::runtime_error("cannot open output");
    output << std::setprecision(17);
    output << "META\t1\t" << robot_model->getName() << '\t' << fixture.anchor_index << '\t'
           << states.size() << '\t' << lower << '\t' << upper << '\t' << SWEEP_STEP_M << '\t'
           << values.size() << '\t' << delta.x() << '\t' << delta.y() << '\t' << delta.z() << '\n';

    bool seen_open_clear = false;
    bool seen_any_desired_contact = false;
    bool seen_dual_contact = false;
    bool forbidden_before_dual = false;
    std::size_t first_open_index = values.size();
    std::size_t first_contact_index = values.size();
    std::size_t first_dual_index = values.size();
    double first_contact_q = 0.0;
    double first_dual_q = 0.0;

    for (std::size_t index = 0; index < values.size(); ++index)
    {
      const double q = values[index];
      moveit::core::RobotState state(robot_model);
      state.setToDefaultValues();
      state.setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
      state.setVariablePosition("gripper_left_joint", q);
      state.update();
      const double right_q = state.getVariablePosition("gripper_right_joint");
      const bool bounds_ok = state.satisfiesBounds(0.0);

      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(request, self_result, state, original_acm);
      const auto self_pairs = contact_pairs(self_result);

      const auto cube_result = check_object(
        scene, state, fixture_acm, fixture.objects[0].name,
        fixture.objects[0].shape, object_poses[0], request);
      const auto cube_pairs = contact_pairs(cube_result);
      const auto support_result = check_object(
        scene, state, fixture_acm, fixture.objects[1].name,
        fixture.objects[1].shape, object_poses[1], request);
      const auto support_pairs = contact_pairs(support_result);
      const auto ground_result = check_object(
        scene, state, ground_acm, "ground_plane", ground_shape,
        Eigen::Isometry3d::Identity(), request);
      const auto ground_pairs = contact_pairs(ground_result);

      const bool left_contact = cube_pairs.count("gripper_left_link|rubiks_cube") != 0;
      const bool right_contact = cube_pairs.count("gripper_right_link|rubiks_cube") != 0;
      bool forbidden_cube = false;
      for (const auto& pair : cube_pairs)
        if (allowed_cube_pairs.count(pair) == 0)
          forbidden_cube = true;
      const bool forbidden = !bounds_ok || self_result.collision || forbidden_cube ||
        support_result.collision || ground_result.collision;
      const bool desired_contact = left_contact || right_contact;
      const bool dual_contact = left_contact && right_contact;
      std::string classification = "OPEN_CLEAR";
      if (forbidden)
        classification = "FORBIDDEN";
      else if (dual_contact)
        classification = "DUAL_FINGER_CUBE";
      else if (left_contact)
        classification = "LEFT_FINGER_CUBE";
      else if (right_contact)
        classification = "RIGHT_FINGER_CUBE";

      if (!forbidden && !desired_contact && !seen_open_clear)
      {
        seen_open_clear = true;
        first_open_index = index;
      }
      if (!forbidden && desired_contact && !seen_any_desired_contact)
      {
        seen_any_desired_contact = true;
        first_contact_index = index;
        first_contact_q = q;
      }
      if (!forbidden && dual_contact && !seen_dual_contact)
      {
        seen_dual_contact = true;
        first_dual_index = index;
        first_dual_q = q;
      }
      if (forbidden && !seen_dual_contact)
        forbidden_before_dual = true;

      output << "STATE\t" << index << '\t' << q << '\t' << right_q << '\t'
             << classification << '\t' << (bounds_ok ? "true" : "false") << '\t'
             << (self_result.collision ? "true" : "false") << '\t'
             << (cube_result.collision ? "true" : "false") << '\t'
             << (support_result.collision ? "true" : "false") << '\t'
             << (ground_result.collision ? "true" : "false") << '\t'
             << (left_contact ? "true" : "false") << '\t'
             << (right_contact ? "true" : "false") << '\t'
             << (forbidden_cube ? "true" : "false") << '\t'
             << join_set(cube_pairs) << '\t' << join_set(support_pairs) << '\t'
             << join_set(ground_pairs) << '\t' << join_set(self_pairs) << '\n';
    }

    std::string decision;
    if (!seen_open_clear)
      decision = "BLOCKED_NO_COLLISION_FREE_OPENING_STATE";
    else if (forbidden_before_dual)
      decision = "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT";
    else if (!seen_dual_contact)
      decision = "BLOCKED_NO_DUAL_FINGER_CUBE_CONTACT_IN_LIMITS";
    else
      decision = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND";

    output << "DECISION\t" << decision << '\t'
           << (seen_open_clear ? "true" : "false") << '\t' << first_open_index << '\t'
           << (seen_any_desired_contact ? "true" : "false") << '\t' << first_contact_index << '\t'
           << first_contact_q << '\t' << (seen_dual_contact ? "true" : "false") << '\t'
           << first_dual_index << '\t' << first_dual_q << '\t'
           << (forbidden_before_dual ? "true" : "false") << '\n';
    output.flush();
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
