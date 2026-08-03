#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
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
  double left_gripper = 0.0;
  double right_gripper = 0.0;
};

struct FixtureObject
{
  std::string name;
  Eigen::Vector3d size;
  Eigen::Vector3d offset_link7;
};

struct Fixture
{
  std::string reference_link;
  std::size_t anchor_index = 0;
  std::size_t candidate_start_index = 0;
  std::size_t state_count = 0;
  std::vector<FixtureObject> objects;
};

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
  const unsigned long long value = std::stoull(text, &consumed);
  if (consumed != text.size())
    throw std::runtime_error("invalid integer: " + text);
  return static_cast<std::size_t>(value);
}

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
      throw std::runtime_error("invalid state field count: " + line);
    State state;
    state.label = fields[0];
    const Eigen::Vector3d translation(number(fields[1]), number(fields[2]), number(fields[3]));
    Eigen::Quaterniond orientation(number(fields[7]), number(fields[4]), number(fields[5]), number(fields[6]));
    if (orientation.norm() < 1e-12)
      throw std::runtime_error("zero root quaternion: " + state.label);
    orientation.normalize();
    state.world_root.linear() = orientation.toRotationMatrix();
    state.world_root.translation() = translation;
    for (std::size_t index = 8; index <= 13; ++index)
      state.arm.push_back(number(fields[index]));
    state.left_gripper = number(fields[14]);
    state.right_gripper = number(fields[15]);
    states.push_back(std::move(state));
  }
  if (states.empty())
    throw std::runtime_error("state matrix is empty");
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    std::ostringstream expected;
    expected << "sample_" << std::setw(6) << std::setfill('0') << index;
    if (states[index].label != expected.str())
      throw std::runtime_error("state label mismatch at " + std::to_string(index));
  }
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
        throw std::runtime_error("invalid fixture META row");
      fixture.reference_link = fields[2];
      fixture.anchor_index = integer(fields[3]);
      fixture.candidate_start_index = integer(fields[4]);
      fixture.state_count = integer(fields[5]);
    }
    else if (fields[0] == "OBJECT")
    {
      if (fields.size() != 8)
        throw std::runtime_error("invalid fixture OBJECT row");
      FixtureObject object;
      object.name = fields[1];
      object.size = Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
      object.offset_link7 = Eigen::Vector3d(number(fields[5]), number(fields[6]), number(fields[7]));
      if ((object.size.array() <= 0.0).any())
        throw std::runtime_error("nonpositive fixture size");
      fixture.objects.push_back(std::move(object));
    }
    else
      throw std::runtime_error("unknown fixture row");
  }
  if (fixture.reference_link != "link7" || fixture.objects.size() != 2)
    throw std::runtime_error("fixture contract incomplete");
  if (fixture.objects[0].name != "rubiks_cube" || fixture.objects[1].name != "rubiks_support")
    throw std::runtime_error("fixture object order mismatch");
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
  const FixtureObject& object,
  const Eigen::Isometry3d& root_object,
  const collision_detection::CollisionRequest& request)
{
  scene.getWorldNonConst()->clearObjects();
  shapes::ShapeConstPtr shape(new shapes::Box(object.size.x(), object.size.y(), object.size.z()));
  scene.getWorldNonConst()->addToObject(object.name, shape, root_object);
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state, acm);
  return result;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 6)
      throw std::runtime_error("usage: fixture_preflight URDF SRDF STATES FIXTURE OUTPUT");

    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
      throw std::runtime_error("URDF parsing failed");
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
      throw std::runtime_error("SRDF parsing failed");
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    if (arm_group == nullptr || robot_model->getLinkModel("link7") == nullptr)
      throw std::runtime_error("required arm/link7 model is missing");
    const std::vector<std::string> expected_arm = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    if (arm_group->getActiveJointModelNames() != expected_arm)
      throw std::runtime_error("active arm joint order mismatch");
    if (robot_model->getRootLinkName() != "base_footprint")
      throw std::runtime_error("unexpected robot root link");

    const auto states = read_states(argv[3]);
    const auto fixture = read_fixture(argv[4]);
    if (fixture.state_count != states.size() || fixture.anchor_index >= states.size() ||
        fixture.candidate_start_index > fixture.anchor_index)
      throw std::runtime_error("fixture/state count or anchor mismatch");

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

    moveit::core::RobotState anchor_state(robot_model);
    anchor_state.setToDefaultValues();
    anchor_state.setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
    anchor_state.setVariablePosition("gripper_left_joint", states[fixture.anchor_index].left_gripper);
    anchor_state.update();
    const Eigen::Isometry3d root_link7_anchor = anchor_state.getGlobalLinkTransform("link7");
    const Eigen::Isometry3d world_link7_anchor =
      states[fixture.anchor_index].world_root * root_link7_anchor;

    std::vector<Eigen::Isometry3d> world_objects;
    for (const auto& object : fixture.objects)
    {
      Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
      link7_object.translation() = object.offset_link7;
      world_objects.push_back(world_link7_anchor * link7_object);
    }

    collision_detection::CollisionRequest self_request;
    self_request.contacts = true;
    self_request.max_contacts = 1000;
    self_request.max_contacts_per_pair = 100;
    collision_detection::CollisionRequest fixture_request = self_request;
    fixture_request.distance = false;

    std::ofstream output(argv[5]);
    if (!output)
      throw std::runtime_error("cannot open output TSV");
    output << "META\t" << robot_model->getName() << "\t" << robot_model->getRootLinkName()
           << "\t" << states.size() << "\t" << fixture.anchor_index << "\t"
           << fixture.candidate_start_index << "\t" << fixture.reference_link << "\t"
           << fixture.objects[0].name << "\t" << fixture.objects[1].name << "\n";
    output << std::setprecision(17);

    for (std::size_t index = 0; index < states.size(); ++index)
    {
      moveit::core::RobotState state(robot_model);
      state.setToDefaultValues();
      state.setJointGroupPositions(arm_group, states[index].arm);
      state.setVariablePosition("gripper_left_joint", states[index].left_gripper);
      state.update();
      const bool bounds_ok = state.satisfiesBounds(0.0);

      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(self_request, self_result, state, original_acm);
      const auto self_pairs = contact_pairs(self_result);

      const Eigen::Isometry3d root_world = states[index].world_root.inverse();
      const auto cube_result = check_object(
        scene, state, fixture_acm, fixture.objects[0], root_world * world_objects[0], fixture_request);
      const auto cube_pairs = contact_pairs(cube_result);
      const auto support_result = check_object(
        scene, state, fixture_acm, fixture.objects[1], root_world * world_objects[1], fixture_request);
      const auto support_pairs = contact_pairs(support_result);

      const char* phase = index < fixture.candidate_start_index ? "approach" :
        (index <= fixture.anchor_index ? "candidate_hold" : "return_and_zero_hold");
      output << "STATE\t" << states[index].label << "\t" << phase << "\t"
             << (bounds_ok ? "true" : "false") << "\t"
             << (self_result.collision ? "true" : "false") << "\t"
             << self_pairs.size() << "\t" << join_set(self_pairs) << "\t"
             << (cube_result.collision ? "true" : "false") << "\t"
             << cube_pairs.size() << "\t" << join_set(cube_pairs) << "\t"
             << (support_result.collision ? "true" : "false") << "\t"
             << support_pairs.size() << "\t" << join_set(support_pairs) << "\n";
      if (index == 0 || index % 100 == 0)
        output.flush();
    }
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
