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
constexpr std::size_t EXPECTED_CANDIDATES = 4896;

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

long signed_integer(const std::string& text)
{
  std::size_t consumed = 0;
  const long value = std::stol(text, &consumed);
  if (consumed != text.size())
    throw std::runtime_error("invalid signed integer: " + text);
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

struct State
{
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

struct Candidate
{
  std::size_t rank = 0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double distance = 0.0;
  long dx = 0;
  long dy = 0;
  long dz = 0;
};

struct SweepState
{
  double q = 0.0;
  std::unique_ptr<moveit::core::RobotState> robot;
  bool base_forbidden = false;
};

struct Result
{
  std::string decision;
  bool open_clear = false;
  bool any_contact = false;
  bool dual_contact = false;
  bool forbidden_before_dual = false;
  double first_contact_q = 0.0;
  double first_dual_q = 0.0;
  std::size_t states_checked = 0;
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
    Eigen::Quaterniond orientation(
      number(fields[7]), number(fields[4]), number(fields[5]), number(fields[6]));
    if (orientation.norm() < TOLERANCE)
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

std::vector<Candidate> read_candidates(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  std::vector<Candidate> candidates;
  bool meta_seen = false;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields[0] == "META")
    {
      if (fields.size() != 10 || fields[1] != "1" || fields[2] != "XYZ_REMAINING" ||
          integer(fields[5]) != EXPECTED_CANDIDATES || integer(fields[6]) != 17)
        throw std::runtime_error("invalid 3-D candidate META");
      const Eigen::Vector3d center(number(fields[7]), number(fields[8]), number(fields[9]));
      if ((center - Eigen::Vector3d(0.06325, 0.0, 0.01225)).norm() > TOLERANCE)
        throw std::runtime_error("3-D candidate center mismatch");
      meta_seen = true;
      continue;
    }
    if (fields.size() != 9 || fields[0] != "CANDIDATE")
      throw std::runtime_error("invalid 3-D candidate row");
    Candidate candidate;
    candidate.rank = integer(fields[1]);
    candidate.translation = Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
    candidate.distance = number(fields[5]);
    candidate.dx = signed_integer(fields[6]);
    candidate.dy = signed_integer(fields[7]);
    candidate.dz = signed_integer(fields[8]);
    if (candidate.dy == 0 && candidate.dz == 0)
      throw std::runtime_error("already-tested X-line candidate leaked into 3-D batch");
    candidates.push_back(candidate);
  }
  if (!meta_seen || candidates.size() != EXPECTED_CANDIDATES)
    throw std::runtime_error("3-D candidate matrix incomplete");
  for (std::size_t index = 0; index < candidates.size(); ++index)
    if (candidates[index].rank != index)
      throw std::runtime_error("candidate ranks are not contiguous");
  return candidates;
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
  std::vector<double> values;
  const std::size_t steps = static_cast<std::size_t>(
    std::floor((upper - lower) / SWEEP_STEP_M + TOLERANCE));
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

Result evaluate_candidate(
  planning_scene::PlanningScene& scene,
  const Fixture& fixture,
  const Candidate& candidate,
  const std::vector<SweepState>& sweep,
  const Eigen::Isometry3d& root_link7,
  const collision_detection::AllowedCollisionMatrix& fixture_acm,
  const collision_detection::CollisionRequest& request)
{
  std::vector<Eigen::Isometry3d> object_poses;
  for (const auto& object : fixture.objects)
  {
    Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
    // Same effective placement as PR28's shifted-fixture + CENTER invocation.
    link7_object.translation() = object.offset_link7 + candidate.translation;
    object_poses.push_back(root_link7 * link7_object);
  }

  const std::set<std::string> allowed_cube_pairs = {
    "gripper_left_link|rubiks_cube", "gripper_right_link|rubiks_cube"
  };
  Result result;
  for (std::size_t index = 0; index < sweep.size(); ++index)
  {
    const auto& item = sweep[index];
    const auto cube_result = check_object(
      scene, *item.robot, fixture_acm, fixture.objects[0].name,
      fixture.objects[0].shape, object_poses[0], request);
    const auto cube_pairs = contact_pairs(cube_result);
    const auto support_result = check_object(
      scene, *item.robot, fixture_acm, fixture.objects[1].name,
      fixture.objects[1].shape, object_poses[1], request);

    const bool left_contact = cube_pairs.count("gripper_left_link|rubiks_cube") != 0;
    const bool right_contact = cube_pairs.count("gripper_right_link|rubiks_cube") != 0;
    bool forbidden_cube = false;
    for (const auto& pair : cube_pairs)
      if (allowed_cube_pairs.count(pair) == 0)
        forbidden_cube = true;
    const bool desired_contact = left_contact || right_contact;
    const bool dual_contact = left_contact && right_contact;
    const bool forbidden = item.base_forbidden || forbidden_cube || support_result.collision;
    result.states_checked = index + 1;

    if (!forbidden && !desired_contact)
      result.open_clear = true;
    if (!forbidden && desired_contact && !result.any_contact)
    {
      result.any_contact = true;
      result.first_contact_q = item.q;
    }
    if (!forbidden && dual_contact && !result.dual_contact)
    {
      result.dual_contact = true;
      result.first_dual_q = item.q;
    }

    // Once a forbidden state occurs before dual contact, the PR27 decision can
    // never recover. Likewise, open-clear followed by valid dual contact is a
    // final positive verdict; later tighter closing states are irrelevant.
    if (forbidden && !result.dual_contact)
    {
      result.forbidden_before_dual = true;
      result.decision = "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT";
      return result;
    }
    if (result.open_clear && result.dual_contact)
    {
      result.decision = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND";
      return result;
    }
  }

  if (!result.open_clear)
    result.decision = "BLOCKED_NO_COLLISION_FREE_OPENING_STATE";
  else if (result.forbidden_before_dual)
    result.decision = "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT";
  else if (!result.dual_contact)
    result.decision = "BLOCKED_NO_DUAL_FINGER_CUBE_CONTACT_IN_LIMITS";
  else
    result.decision = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND";
  return result;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 8)
      throw std::runtime_error(
        "usage: gripper_sweep_batch URDF SRDF STATES FIXTURE CANDIDATES SCAN COMPATIBLE");

    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
      throw std::runtime_error("URDF parsing failed");
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
      throw std::runtime_error("SRDF parsing failed");
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    if (arm_group == nullptr || robot_model->getLinkModel("link7") == nullptr ||
        robot_model->getRootLinkName() != "base_footprint")
      throw std::runtime_error("robot model identity mismatch");

    const auto states = read_states(argv[3]);
    const auto fixture = read_fixture(argv[4]);
    const auto candidates = read_candidates(argv[5]);
    if (fixture.state_count != states.size() || fixture.anchor_index >= states.size())
      throw std::runtime_error("fixture/state identity mismatch");

    const auto left_joint = urdf_model->getJoint("gripper_left_joint");
    const auto right_joint = urdf_model->getJoint("gripper_right_joint");
    if (!left_joint || !right_joint || !left_joint->limits || !right_joint->mimic)
      throw std::runtime_error("gripper joint contract missing");
    const double lower = left_joint->limits->lower;
    const double upper = left_joint->limits->upper;
    if (std::abs(lower + 0.01) > TOLERANCE || std::abs(upper - 0.019) > TOLERANCE ||
        right_joint->mimic->joint_name != "gripper_left_joint" ||
        std::abs(right_joint->mimic->multiplier - 1.0) > TOLERANCE)
      throw std::runtime_error("gripper limit/mimic contract mismatch");

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

    const Eigen::Matrix3d root_rotation_world = states[fixture.anchor_index].world_root.linear();
    const Eigen::Vector3d normal_root = root_rotation_world.transpose() * Eigen::Vector3d::UnitZ();
    const double ground_offset_root = states[fixture.anchor_index].world_root.translation().z();
    shapes::ShapeConstPtr ground_shape(new shapes::Plane(
      normal_root.x(), normal_root.y(), normal_root.z(), ground_offset_root));
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

    moveit::core::RobotState anchor(robot_model);
    anchor.setToDefaultValues();
    anchor.setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
    anchor.setVariablePosition("gripper_left_joint", upper);
    anchor.update();
    const Eigen::Isometry3d root_link7 = anchor.getGlobalLinkTransform("link7");

    const auto values = sweep_values(lower, upper);
    std::vector<SweepState> sweep;
    sweep.reserve(values.size());
    for (const double q : values)
    {
      SweepState item;
      item.q = q;
      item.robot = std::make_unique<moveit::core::RobotState>(robot_model);
      item.robot->setToDefaultValues();
      item.robot->setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
      item.robot->setVariablePosition("gripper_left_joint", q);
      item.robot->update();
      const bool bounds_ok = item.robot->satisfiesBounds(0.0);
      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(request, self_result, *item.robot, original_acm);
      const auto ground_result = check_object(
        scene, *item.robot, ground_acm, "ground_plane", ground_shape,
        Eigen::Isometry3d::Identity(), request);
      item.base_forbidden = !bounds_ok || self_result.collision || ground_result.collision;
      sweep.push_back(std::move(item));
    }

    std::vector<Result> results;
    results.reserve(candidates.size());
    std::size_t compatible_count = 0;
    std::size_t total_checks = 0;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      auto result = evaluate_candidate(
        scene, fixture, candidates[index], sweep, root_link7, fixture_acm, request);
      total_checks += result.states_checked;
      if (result.decision == "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND")
        ++compatible_count;
      results.push_back(std::move(result));
      if ((index + 1) % 100 == 0 || index + 1 == candidates.size())
        std::cout << "completed=" << (index + 1) << '/' << candidates.size()
                  << " compatible=" << compatible_count << '\n' << std::flush;
    }

    std::ofstream scan_output(argv[6]);
    std::ofstream compatible_output(argv[7]);
    if (!scan_output || !compatible_output)
      throw std::runtime_error("cannot open batch output");
    scan_output << std::setprecision(17);
    compatible_output << std::setprecision(17);
    scan_output << "META\t1\tXYZ_REMAINING\t" << candidates.size() << '\t'
                << compatible_count << '\t' << total_checks << '\n';
    compatible_output << "META\t1\tXYZ_COMPATIBLE\t" << compatible_count << '\n';

    std::size_t compatible_rank = 0;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      const auto& candidate = candidates[index];
      const auto& result = results[index];
      scan_output << "SCAN\t" << candidate.rank << '\t'
                  << candidate.translation.x() << '\t' << candidate.translation.y() << '\t'
                  << candidate.translation.z() << '\t' << candidate.distance << '\t'
                  << candidate.dx << '\t' << candidate.dy << '\t' << candidate.dz << '\t'
                  << result.decision << '\t' << (result.open_clear ? "true" : "false") << '\t'
                  << result.first_contact_q << '\t' << (result.dual_contact ? "true" : "false") << '\t'
                  << result.first_dual_q << '\t'
                  << (result.forbidden_before_dual ? "true" : "false") << '\t'
                  << result.states_checked << '\n';
      if (result.decision == "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND")
      {
        const double norm = candidate.translation.norm();
        compatible_output << "CANDIDATE\t" << compatible_rank++ << '\t'
                          << candidate.translation.x() << '\t' << candidate.translation.y() << '\t'
                          << candidate.translation.z() << '\t' << norm << '\t'
                          << candidate.dx << '\t' << candidate.dy << '\t' << candidate.dz
                          << "\t0\t0\t0\n";
      }
    }
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
