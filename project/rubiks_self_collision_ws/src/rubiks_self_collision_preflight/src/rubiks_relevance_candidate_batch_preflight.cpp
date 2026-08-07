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
#include <map>
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
  double left_gripper = 0.0;
};

struct PreparedState
{
  std::string label;
  std::string phase;
  Eigen::Isometry3d root_world = Eigen::Isometry3d::Identity();
  std::unique_ptr<moveit::core::RobotState> robot;
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
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  double norm = 0.0;
};

struct Evaluation
{
  bool clear = false;
  std::size_t states_checked = 0;
  std::size_t first_collision_index = 0;
  std::string first_collision_phase = "none";
  bool cube_collision = false;
  bool support_collision = false;
  std::set<std::string> pairs;
};

std::string phase_for_index(std::size_t index, const Fixture& fixture)
{
  if (index < fixture.candidate_start_index)
    return "approach";
  if (index <= fixture.anchor_index)
    return "candidate_hold";
  return "return_and_zero_hold";
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
      throw std::runtime_error("invalid state field count");
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
    state.left_gripper = number(fields[14]);
    static_cast<void>(number(fields[15]));
    states.push_back(std::move(state));
  }
  if (states.empty())
    throw std::runtime_error("state matrix is empty");
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    std::ostringstream expected;
    expected << "sample_" << std::setw(6) << std::setfill('0') << index;
    if (states[index].label != expected.str())
      throw std::runtime_error("state label mismatch");
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
    throw std::runtime_error("fixture contract incomplete");
  return fixture;
}

std::vector<Candidate> read_candidates(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  std::vector<Candidate> candidates;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields[0] == "META")
      continue;
    if (fields[0] != "CANDIDATE" || fields.size() != 12)
      throw std::runtime_error("invalid candidate row");
    Candidate candidate;
    candidate.rank = integer(fields[1]);
    candidate.delta = Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
    candidate.norm = number(fields[5]);
    if (std::abs(candidate.norm - candidate.delta.norm()) > 1e-12)
      throw std::runtime_error("candidate norm mismatch");
    candidates.push_back(candidate);
  }
  if (candidates.empty())
    throw std::runtime_error("candidate list is empty");
  for (std::size_t index = 0; index < candidates.size(); ++index)
    if (candidates[index].rank != index)
      throw std::runtime_error("candidate rank mismatch");
  return candidates;
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
  scene.getWorldNonConst()->addToObject(object.name, object.shape, root_object);
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state, acm);
  return result;
}

std::vector<PreparedState> prepare_states(
  const std::vector<State>& states,
  const Fixture& fixture,
  const moveit::core::RobotModelConstPtr& robot_model,
  const moveit::core::JointModelGroup* arm_group,
  planning_scene::PlanningScene& scene,
  const collision_detection::AllowedCollisionMatrix& original_acm)
{
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 1000;
  request.max_contacts_per_pair = 100;
  std::vector<PreparedState> prepared;
  prepared.reserve(states.size());
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    auto robot = std::make_unique<moveit::core::RobotState>(robot_model);
    robot->setToDefaultValues();
    robot->setJointGroupPositions(arm_group, states[index].arm);
    robot->setVariablePosition("gripper_left_joint", states[index].left_gripper);
    robot->update();
    if (!robot->satisfiesBounds(0.0))
      throw std::runtime_error("accepted state out of bounds");
    collision_detection::CollisionResult self_result;
    scene.checkSelfCollision(request, self_result, *robot, original_acm);
    if (self_result.collision)
      throw std::runtime_error("accepted state self-collides");
    PreparedState item;
    item.label = states[index].label;
    item.phase = phase_for_index(index, fixture);
    item.root_world = states[index].world_root.inverse();
    item.robot = std::move(robot);
    prepared.push_back(std::move(item));
  }
  return prepared;
}

std::vector<Eigen::Isometry3d> world_object_poses(
  const Fixture& fixture,
  const Eigen::Isometry3d& world_link7_anchor,
  const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Isometry3d> poses;
  for (const auto& object : fixture.objects)
  {
    Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
    link7_object.translation() = object.offset_link7 + delta;
    poses.push_back(world_link7_anchor * link7_object);
  }
  return poses;
}

Evaluation evaluate(
  planning_scene::PlanningScene& scene,
  const std::vector<PreparedState>& states,
  const Fixture& fixture,
  const Eigen::Isometry3d& world_link7_anchor,
  const Eigen::Vector3d& delta,
  const collision_detection::AllowedCollisionMatrix& fixture_acm,
  const collision_detection::CollisionRequest& request)
{
  const auto poses = world_object_poses(fixture, world_link7_anchor, delta);
  Evaluation result;
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    const auto cube = check_object(
      scene, *states[index].robot, fixture_acm, fixture.objects[0],
      states[index].root_world * poses[0], request);
    const auto support = check_object(
      scene, *states[index].robot, fixture_acm, fixture.objects[1],
      states[index].root_world * poses[1], request);
    result.states_checked = index + 1;
    if (!cube.collision && !support.collision)
      continue;
    result.first_collision_index = index;
    result.first_collision_phase = states[index].phase;
    result.cube_collision = cube.collision;
    result.support_collision = support.collision;
    const auto cube_pairs = contact_pairs(cube);
    const auto support_pairs = contact_pairs(support);
    result.pairs.insert(cube_pairs.begin(), cube_pairs.end());
    result.pairs.insert(support_pairs.begin(), support_pairs.end());
    if (result.pairs.empty())
      throw std::runtime_error("collision without contact pair");
    return result;
  }
  result.clear = true;
  result.first_collision_index = states.size();
  return result;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 7)
      throw std::runtime_error("usage: relevance_batch URDF SRDF STATES FIXTURE CANDIDATES OUTPUT");
    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
      throw std::runtime_error("URDF parsing failed");
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
      throw std::runtime_error("SRDF parsing failed");
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    if (arm_group == nullptr || robot_model->getLinkModel("link7") == nullptr)
      throw std::runtime_error("required arm/link7 model missing");
    const std::vector<std::string> expected_arm = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    if (arm_group->getActiveJointModelNames() != expected_arm ||
        robot_model->getRootLinkName() != "base_footprint")
      throw std::runtime_error("robot model identity mismatch");

    const auto states = read_states(argv[3]);
    const auto fixture = read_fixture(argv[4]);
    const auto candidates = read_candidates(argv[5]);
    if (fixture.state_count != states.size() || fixture.anchor_index >= states.size())
      throw std::runtime_error("fixture/state mismatch");

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

    const auto prepared = prepare_states(states, fixture, robot_model, arm_group, scene, original_acm);
    const Eigen::Isometry3d root_link7_anchor =
      prepared[fixture.anchor_index].robot->getGlobalLinkTransform("link7");
    const Eigen::Isometry3d world_link7_anchor =
      states[fixture.anchor_index].world_root * root_link7_anchor;

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.distance = false;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    const Evaluation baseline = evaluate(
      scene, prepared, fixture, world_link7_anchor,
      Eigen::Vector3d::Zero(), fixture_acm, request);
    const std::set<std::string> expected_pairs = {
      "gripper_left_link|rubiks_cube", "gripper_left_link|rubiks_support",
      "gripper_right_link|rubiks_cube", "gripper_right_link|rubiks_support"
    };
    if (baseline.clear || baseline.first_collision_index != 0 || baseline.pairs != expected_pairs)
      throw std::runtime_error("baseline PR22 blocker mismatch");

    std::ofstream output(argv[6]);
    if (!output)
      throw std::runtime_error("cannot open output");
    output << std::setprecision(17);
    output << "META\t1\t" << robot_model->getName() << '\t' << states.size() << '\t'
           << candidates.size() << '\t' << fixture.anchor_index << '\n';
    output << "BASELINE\tfalse\t0\t" << join_set(baseline.pairs) << '\n';

    std::size_t total_state_checks = 0;
    bool found = false;
    Candidate selected;
    Evaluation selected_evaluation;
    for (const auto& candidate : candidates)
    {
      const Evaluation evaluation = evaluate(
        scene, prepared, fixture, world_link7_anchor,
        candidate.delta, fixture_acm, request);
      total_state_checks += evaluation.states_checked;
      output << "CANDIDATE\t" << candidate.rank << '\t'
             << candidate.delta.x() << '\t' << candidate.delta.y() << '\t'
             << candidate.delta.z() << '\t' << candidate.norm << '\t'
             << (evaluation.clear ? "true" : "false") << '\t'
             << evaluation.states_checked << '\t' << evaluation.first_collision_index << '\t'
             << evaluation.first_collision_phase << '\t'
             << (evaluation.cube_collision ? "true" : "false") << '\t'
             << (evaluation.support_collision ? "true" : "false") << '\t'
             << join_set(evaluation.pairs) << '\n';
      if (evaluation.clear)
      {
        found = true;
        selected = candidate;
        selected_evaluation = evaluation;
        break;
      }
    }
    output << "SELECT\t" << (found ? "true" : "false") << '\t'
           << selected.delta.x() << '\t' << selected.delta.y() << '\t'
           << selected.delta.z() << '\t' << selected.norm << '\t'
           << total_state_checks << '\t'
           << (found ? selected_evaluation.states_checked : 0) << '\n';
    output.flush();
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
