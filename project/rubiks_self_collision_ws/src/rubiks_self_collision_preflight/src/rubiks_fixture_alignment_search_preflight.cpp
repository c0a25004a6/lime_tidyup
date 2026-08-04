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
#include <tuple>
#include <utility>
#include <vector>

namespace
{
constexpr double COARSE_STEP_M = 0.002;
constexpr int COARSE_RADIUS_STEPS = 40;
constexpr double FINE_STEP_M = 0.00025;
constexpr int FINE_RADIUS_STEPS = 8;

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

std::string phase_for_index(std::size_t index, std::size_t candidate_start, std::size_t anchor)
{
  if (index < candidate_start)
    return "approach";
  if (index <= anchor)
    return "candidate_hold";
  return "return_and_zero_hold";
}

struct State
{
  std::string label;
  Eigen::Isometry3d world_root = Eigen::Isometry3d::Identity();
  std::vector<double> arm;
  double left_gripper = 0.0;
  double right_gripper = 0.0;
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

struct Evaluation
{
  bool clear = false;
  std::size_t states_checked = 0;
  std::size_t first_collision_index = 0;
  std::string first_collision_phase;
  bool cube_collision = false;
  bool support_collision = false;
  std::set<std::string> pairs;
};

struct SearchStats
{
  std::size_t candidates_checked = 0;
  std::size_t total_state_checks = 0;
  std::size_t maximum_states_checked = 0;
  std::size_t rejected_cube_only = 0;
  std::size_t rejected_support_only = 0;
  std::size_t rejected_both = 0;
  std::map<std::string, std::size_t> rejected_by_phase = {
    {"approach", 0}, {"candidate_hold", 0}, {"return_and_zero_hold", 0}
  };
};

struct CoarseCandidate
{
  int x = 0;
  int y = 0;
  int z = 0;
  int norm2 = 0;
};

struct FineCandidate
{
  int x_units = 0;
  int y_units = 0;
  int z_units = 0;
  int norm2_units = 0;
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
      object.shape.reset(new shapes::Box(object.size.x(), object.size.y(), object.size.z()));
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

bool pair_contains(const std::string& pair, const std::string& object)
{
  const auto separator = pair.find('|');
  if (separator == std::string::npos)
    return false;
  return pair.substr(0, separator) == object || pair.substr(separator + 1) == object;
}

std::vector<PreparedState> prepare_states(
  const std::vector<State>& states,
  const Fixture& fixture,
  const moveit::core::RobotModelConstPtr& robot_model,
  const moveit::core::JointModelGroup* arm_group,
  planning_scene::PlanningScene& scene,
  const collision_detection::AllowedCollisionMatrix& original_acm)
{
  collision_detection::CollisionRequest self_request;
  self_request.contacts = true;
  self_request.max_contacts = 1000;
  self_request.max_contacts_per_pair = 100;

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
      throw std::runtime_error("accepted state is out of bounds: " + states[index].label);
    collision_detection::CollisionResult self_result;
    scene.checkSelfCollision(self_request, self_result, *robot, original_acm);
    if (self_result.collision)
      throw std::runtime_error("accepted state self-collides: " + states[index].label);
    PreparedState item;
    item.label = states[index].label;
    item.phase = phase_for_index(index, fixture.candidate_start_index, fixture.anchor_index);
    item.root_world = states[index].world_root.inverse();
    item.robot = std::move(robot);
    prepared.push_back(std::move(item));
  }
  return prepared;
}

Evaluation evaluate_translation(
  planning_scene::PlanningScene& scene,
  const std::vector<PreparedState>& states,
  const Fixture& fixture,
  const Eigen::Isometry3d& world_link7_anchor,
  const Eigen::Vector3d& delta_link7,
  const collision_detection::AllowedCollisionMatrix& fixture_acm,
  const collision_detection::CollisionRequest& request)
{
  std::vector<Eigen::Isometry3d> world_objects;
  world_objects.reserve(fixture.objects.size());
  for (const auto& object : fixture.objects)
  {
    Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
    link7_object.translation() = object.offset_link7 + delta_link7;
    world_objects.push_back(world_link7_anchor * link7_object);
  }

  Evaluation evaluation;
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    scene.getWorldNonConst()->clearObjects();
    for (std::size_t object_index = 0; object_index < fixture.objects.size(); ++object_index)
    {
      scene.getWorldNonConst()->addToObject(
        fixture.objects[object_index].name,
        fixture.objects[object_index].shape,
        states[index].root_world * world_objects[object_index]);
    }
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, *states[index].robot, fixture_acm);
    evaluation.states_checked = index + 1;
    if (!result.collision)
      continue;
    evaluation.clear = false;
    evaluation.first_collision_index = index;
    evaluation.first_collision_phase = states[index].phase;
    evaluation.pairs = contact_pairs(result);
    if (evaluation.pairs.empty())
      throw std::runtime_error("collision result lacks contact pairs");
    for (const auto& pair : evaluation.pairs)
    {
      evaluation.cube_collision = evaluation.cube_collision || pair_contains(pair, fixture.objects[0].name);
      evaluation.support_collision = evaluation.support_collision || pair_contains(pair, fixture.objects[1].name);
    }
    if (!evaluation.cube_collision && !evaluation.support_collision)
      throw std::runtime_error("collision pair does not reference fixture objects");
    return evaluation;
  }
  evaluation.clear = true;
  evaluation.first_collision_index = states.size();
  evaluation.first_collision_phase = "none";
  return evaluation;
}

void record_rejection(SearchStats& stats, const Evaluation& evaluation)
{
  ++stats.candidates_checked;
  stats.total_state_checks += evaluation.states_checked;
  stats.maximum_states_checked = std::max(stats.maximum_states_checked, evaluation.states_checked);
  if (evaluation.clear)
    return;
  if (evaluation.cube_collision && evaluation.support_collision)
    ++stats.rejected_both;
  else if (evaluation.cube_collision)
    ++stats.rejected_cube_only;
  else if (evaluation.support_collision)
    ++stats.rejected_support_only;
  ++stats.rejected_by_phase.at(evaluation.first_collision_phase);
}

std::vector<CoarseCandidate> coarse_candidates()
{
  std::vector<CoarseCandidate> candidates;
  const int radius2 = COARSE_RADIUS_STEPS * COARSE_RADIUS_STEPS;
  for (int x = -COARSE_RADIUS_STEPS; x <= COARSE_RADIUS_STEPS; ++x)
    for (int y = -COARSE_RADIUS_STEPS; y <= COARSE_RADIUS_STEPS; ++y)
      for (int z = -COARSE_RADIUS_STEPS; z <= COARSE_RADIUS_STEPS; ++z)
      {
        const int norm2 = x * x + y * y + z * z;
        if (norm2 <= radius2)
          candidates.push_back({x, y, z, norm2});
      }
  std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
    return std::make_tuple(
      left.norm2, std::abs(left.z), std::abs(left.y), std::abs(left.x), left.z, left.y, left.x) <
      std::make_tuple(
        right.norm2, std::abs(right.z), std::abs(right.y), std::abs(right.x), right.z, right.y, right.x);
  });
  return candidates;
}

std::vector<FineCandidate> fine_candidates(const CoarseCandidate& center)
{
  std::vector<FineCandidate> candidates;
  const int center_x = center.x * static_cast<int>(std::llround(COARSE_STEP_M / FINE_STEP_M));
  const int center_y = center.y * static_cast<int>(std::llround(COARSE_STEP_M / FINE_STEP_M));
  const int center_z = center.z * static_cast<int>(std::llround(COARSE_STEP_M / FINE_STEP_M));
  for (int x = -FINE_RADIUS_STEPS; x <= FINE_RADIUS_STEPS; ++x)
    for (int y = -FINE_RADIUS_STEPS; y <= FINE_RADIUS_STEPS; ++y)
      for (int z = -FINE_RADIUS_STEPS; z <= FINE_RADIUS_STEPS; ++z)
      {
        const int absolute_x = center_x + x;
        const int absolute_y = center_y + y;
        const int absolute_z = center_z + z;
        candidates.push_back({
          absolute_x,
          absolute_y,
          absolute_z,
          absolute_x * absolute_x + absolute_y * absolute_y + absolute_z * absolute_z
        });
      }
  std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
    return std::make_tuple(
      left.norm2_units,
      std::abs(left.z_units), std::abs(left.y_units), std::abs(left.x_units),
      left.z_units, left.y_units, left.x_units) <
      std::make_tuple(
        right.norm2_units,
        std::abs(right.z_units), std::abs(right.y_units), std::abs(right.x_units),
        right.z_units, right.y_units, right.x_units);
  });
  return candidates;
}

Eigen::Vector3d coarse_translation(const CoarseCandidate& candidate)
{
  return Eigen::Vector3d(
    candidate.x * COARSE_STEP_M,
    candidate.y * COARSE_STEP_M,
    candidate.z * COARSE_STEP_M);
}

Eigen::Vector3d fine_translation(const FineCandidate& candidate)
{
  return Eigen::Vector3d(
    candidate.x_units * FINE_STEP_M,
    candidate.y_units * FINE_STEP_M,
    candidate.z_units * FINE_STEP_M);
}

void write_search_stats(
  std::ofstream& output,
  const std::string& label,
  double step,
  double radius,
  std::size_t candidate_count,
  const SearchStats& stats,
  bool found,
  const Eigen::Vector3d& selected)
{
  output << label << '\t' << step << '\t' << radius << '\t' << candidate_count << '\t'
         << stats.candidates_checked << '\t' << (found ? "true" : "false") << '\t'
         << selected.x() << '\t' << selected.y() << '\t' << selected.z() << '\t'
         << selected.norm() << '\t' << stats.total_state_checks << '\t'
         << stats.maximum_states_checked << '\t' << stats.rejected_cube_only << '\t'
         << stats.rejected_support_only << '\t' << stats.rejected_both << '\t'
         << stats.rejected_by_phase.at("approach") << '\t'
         << stats.rejected_by_phase.at("candidate_hold") << '\t'
         << stats.rejected_by_phase.at("return_and_zero_hold") << '\n';
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 6)
      throw std::runtime_error("usage: fixture_alignment_search URDF SRDF STATES FIXTURE OUTPUT");

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
    const auto prepared = prepare_states(states, fixture, robot_model, arm_group, scene, original_acm);

    auto fixture_acm = original_acm;
    const auto links = robot_model->getLinkModelNames();
    for (const auto& first : links)
      for (const auto& second : links)
        fixture_acm.setEntry(first, second, true);
    for (const auto& object : fixture.objects)
      for (const auto& link : links)
        fixture_acm.setEntry(object.name, link, false);

    const Eigen::Isometry3d root_link7_anchor =
      prepared[fixture.anchor_index].robot->getGlobalLinkTransform("link7");
    const Eigen::Isometry3d world_link7_anchor =
      states[fixture.anchor_index].world_root * root_link7_anchor;

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.distance = false;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    const Evaluation baseline = evaluate_translation(
      scene, prepared, fixture, world_link7_anchor, Eigen::Vector3d::Zero(), fixture_acm, request);
    if (baseline.clear)
      throw std::runtime_error("baseline unexpectedly became fixture-clear");

    const auto coarse = coarse_candidates();
    SearchStats coarse_stats;
    bool coarse_found = false;
    CoarseCandidate coarse_selected;
    Eigen::Vector3d coarse_delta = Eigen::Vector3d::Zero();
    for (const auto& candidate : coarse)
    {
      const Eigen::Vector3d delta = coarse_translation(candidate);
      const Evaluation evaluation = evaluate_translation(
        scene, prepared, fixture, world_link7_anchor, delta, fixture_acm, request);
      record_rejection(coarse_stats, evaluation);
      if (evaluation.clear)
      {
        coarse_found = true;
        coarse_selected = candidate;
        coarse_delta = delta;
        break;
      }
    }

    SearchStats fine_stats;
    bool fine_found = false;
    Eigen::Vector3d selected_delta = coarse_delta;
    std::size_t fine_candidate_count = 0;
    if (coarse_found)
    {
      const auto fine = fine_candidates(coarse_selected);
      fine_candidate_count = fine.size();
      for (const auto& candidate : fine)
      {
        const Eigen::Vector3d delta = fine_translation(candidate);
        const Evaluation evaluation = evaluate_translation(
          scene, prepared, fixture, world_link7_anchor, delta, fixture_acm, request);
        record_rejection(fine_stats, evaluation);
        if (evaluation.clear)
        {
          fine_found = true;
          selected_delta = delta;
          break;
        }
      }
      if (!fine_found)
        throw std::runtime_error("coarse clear candidate was not retained by local fine grid");
    }

    const bool selected_found = coarse_found && fine_found;
    const std::string decision = selected_found ?
      "FULL_PATH_CLEAR_TRANSLATION_FOUND" :
      "NO_FULL_PATH_CLEAR_TRANSLATION_WITHIN_COARSE_RADIUS";

    std::ofstream output(argv[5]);
    if (!output)
      throw std::runtime_error("cannot open output TSV");
    output << std::setprecision(17);
    output << "META\t1\t" << robot_model->getName() << '\t' << robot_model->getRootLinkName()
           << '\t' << states.size() << '\t' << fixture.anchor_index << '\t'
           << fixture.candidate_start_index << '\t' << fixture.reference_link << '\n';
    output << "BASELINE\tfalse\t" << baseline.first_collision_index << '\t'
           << baseline.first_collision_phase << '\t'
           << (baseline.cube_collision ? "true" : "false") << '\t'
           << (baseline.support_collision ? "true" : "false") << '\t'
           << baseline.states_checked << '\t' << join_set(baseline.pairs) << '\n';
    write_search_stats(
      output,
      "COARSE",
      COARSE_STEP_M,
      COARSE_RADIUS_STEPS * COARSE_STEP_M,
      coarse.size(),
      coarse_stats,
      coarse_found,
      coarse_delta);
    write_search_stats(
      output,
      "FINE",
      FINE_STEP_M,
      FINE_RADIUS_STEPS * FINE_STEP_M,
      fine_candidate_count,
      fine_stats,
      fine_found,
      selected_delta);
    output << "SELECT\t" << (selected_found ? "true" : "false") << '\t'
           << selected_delta.x() << '\t' << selected_delta.y() << '\t'
           << selected_delta.z() << '\t' << selected_delta.norm() << '\t'
           << decision << '\n';
    output.flush();

    if (!selected_found)
      return 0;

    std::vector<Eigen::Isometry3d> world_objects;
    for (const auto& object : fixture.objects)
    {
      Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
      link7_object.translation() = object.offset_link7 + selected_delta;
      world_objects.push_back(world_link7_anchor * link7_object);
    }

    collision_detection::CollisionRequest self_request = request;
    for (std::size_t index = 0; index < prepared.size(); ++index)
    {
      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(self_request, self_result, *prepared[index].robot, original_acm);
      const auto self_pairs = contact_pairs(self_result);

      scene.getWorldNonConst()->clearObjects();
      for (std::size_t object_index = 0; object_index < fixture.objects.size(); ++object_index)
      {
        scene.getWorldNonConst()->addToObject(
          fixture.objects[object_index].name,
          fixture.objects[object_index].shape,
          prepared[index].root_world * world_objects[object_index]);
      }
      collision_detection::CollisionResult result;
      scene.checkCollision(request, result, *prepared[index].robot, fixture_acm);
      const auto pairs = contact_pairs(result);
      bool cube_collision = false;
      bool support_collision = false;
      for (const auto& pair : pairs)
      {
        cube_collision = cube_collision || pair_contains(pair, fixture.objects[0].name);
        support_collision = support_collision || pair_contains(pair, fixture.objects[1].name);
      }
      output << "STATE\t" << prepared[index].label << '\t' << prepared[index].phase << "\ttrue\t"
             << (self_result.collision ? "true" : "false") << '\t' << join_set(self_pairs) << '\t'
             << (cube_collision ? "true" : "false") << '\t'
             << (support_collision ? "true" : "false") << '\t'
             << pairs.size() << '\t' << join_set(pairs) << '\n';
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
