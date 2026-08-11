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
constexpr double TRANSLATION_STEP_M = 0.0005;
constexpr double TOLERANCE = 1e-12;
constexpr double FACE_TOLERANCE_M = 0.0010;
constexpr double EDGE_MARGIN_M = 0.0010;
constexpr double NORMAL_Y_MIN = 0.70;
constexpr double MAX_CONTACT_DEPTH_M = 0.0010;
constexpr double MIN_QUALITY_FRACTION = 0.80;
constexpr std::size_t EXPECTED_CANDIDATE_COUNT = 441;
const Eigen::Vector3d CENTER(0.063, 0.0, 0.01225);

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

long signed_integer(const std::string& text)
{
  std::size_t consumed = 0;
  const auto value = std::stoll(text, &consumed);
  if (consumed != text.size())
    throw std::runtime_error("invalid signed integer: " + text);
  return static_cast<long>(value);
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
  std::size_t state_count = 0;
  std::vector<FixtureObject> objects;
};

struct Candidate
{
  std::size_t rank = 0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double distance = 0.0;
  long dx = 0;
  long dz = 0;
};

struct PreparedSweepState
{
  double q = 0.0;
  std::unique_ptr<moveit::core::RobotState> robot;
  bool static_forbidden = false;
};

struct FingerQuality
{
  std::size_t total = 0;
  std::size_t side_face = 0;
  std::size_t interior = 0;
  std::size_t y_normal_dominant = 0;
  std::size_t qualified = 0;
  double max_depth = 0.0;
  Eigen::Vector3d mean_local_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_abs_local_normal = Eigen::Vector3d::Zero();
};

struct Evaluation
{
  std::string decision;
  bool open_clear = false;
  bool first_dual_found = false;
  bool quality_pass = false;
  bool forbidden_before_dual = false;
  std::size_t first_dual_index = 0;
  double first_dual_q = 0.0;
  FingerQuality left;
  FingerQuality right;
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
  if ((fixture.objects[0].size - Eigen::Vector3d(0.057, 0.057, 0.057)).norm() > TOLERANCE)
    throw std::runtime_error("cube size mismatch");
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
      if (fields.size() != 9 || fields[1] != "1" || fields[2] != "XZ_OPPOSING_SIDE_CONTACT" ||
          std::abs(number(fields[3]) - TRANSLATION_STEP_M) > TOLERANCE ||
          integer(fields[4]) != 10 || integer(fields[5]) != EXPECTED_CANDIDATE_COUNT)
        throw std::runtime_error("invalid XZ candidate META");
      const Eigen::Vector3d center(number(fields[6]), number(fields[7]), number(fields[8]));
      if ((center - CENTER).norm() > TOLERANCE)
        throw std::runtime_error("XZ candidate center mismatch");
      meta_seen = true;
      continue;
    }
    if (fields[0] != "CANDIDATE" || fields.size() != 8)
      throw std::runtime_error("invalid XZ candidate row");
    Candidate candidate;
    candidate.rank = integer(fields[1]);
    candidate.translation = Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
    candidate.distance = number(fields[5]);
    candidate.dx = signed_integer(fields[6]);
    candidate.dz = signed_integer(fields[7]);
    const Eigen::Vector3d expected = CENTER + TRANSLATION_STEP_M *
      Eigen::Vector3d(static_cast<double>(candidate.dx), 0.0, static_cast<double>(candidate.dz));
    if ((candidate.translation - expected).norm() > TOLERANCE)
      throw std::runtime_error("candidate translation/offset mismatch");
    if (std::abs(candidate.distance - TRANSLATION_STEP_M *
        std::sqrt(static_cast<double>(candidate.dx * candidate.dx + candidate.dz * candidate.dz))) > TOLERANCE)
      throw std::runtime_error("candidate distance mismatch");
    candidates.push_back(candidate);
  }
  if (!meta_seen || candidates.size() != EXPECTED_CANDIDATE_COUNT)
    throw std::runtime_error("XZ candidate matrix incomplete");
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

std::vector<PreparedSweepState> prepare_sweep_states(
  const moveit::core::RobotModelConstPtr& robot_model,
  const moveit::core::JointModelGroup* arm_group,
  const State& anchor_state,
  const std::vector<double>& values,
  planning_scene::PlanningScene& scene,
  const collision_detection::AllowedCollisionMatrix& original_acm,
  const collision_detection::AllowedCollisionMatrix& ground_acm,
  const shapes::ShapeConstPtr& ground_shape,
  const collision_detection::CollisionRequest& request)
{
  std::vector<PreparedSweepState> prepared;
  prepared.reserve(values.size());
  for (const double q : values)
  {
    PreparedSweepState item;
    item.q = q;
    item.robot = std::make_unique<moveit::core::RobotState>(robot_model);
    item.robot->setToDefaultValues();
    item.robot->setJointGroupPositions(arm_group, anchor_state.arm);
    item.robot->setVariablePosition("gripper_left_joint", q);
    item.robot->update();

    collision_detection::CollisionResult self_result;
    scene.checkSelfCollision(request, self_result, *item.robot, original_acm);
    scene.getWorldNonConst()->clearObjects();
    scene.getWorldNonConst()->addToObject("ground_plane", ground_shape, Eigen::Isometry3d::Identity());
    collision_detection::CollisionResult ground_result;
    scene.checkCollision(request, ground_result, *item.robot, ground_acm);
    item.static_forbidden = !item.robot->satisfiesBounds(0.0) || self_result.collision || ground_result.collision;
    prepared.push_back(std::move(item));
  }
  scene.getWorldNonConst()->clearObjects();
  return prepared;
}

void accumulate_quality(
  const collision_detection::CollisionResult& collision,
  const Eigen::Isometry3d& root_cube,
  const Eigen::Vector3d& cube_size,
  FingerQuality& left,
  FingerQuality& right)
{
  const Eigen::Isometry3d cube_root = root_cube.inverse();
  const double hx = 0.5 * cube_size.x();
  const double hy = 0.5 * cube_size.y();
  const double hz = 0.5 * cube_size.z();
  for (const auto& entry : collision.contacts)
  {
    const std::string pair = entry.first.first + " " + entry.first.second;
    FingerQuality* target = nullptr;
    bool positive_y = false;
    if (pair.find("gripper_left_link") != std::string::npos && pair.find("rubiks_cube") != std::string::npos)
    {
      target = &left;
      positive_y = true;
    }
    else if (pair.find("gripper_right_link") != std::string::npos && pair.find("rubiks_cube") != std::string::npos)
    {
      target = &right;
      positive_y = false;
    }
    if (target == nullptr)
      continue;

    for (const auto& contact : entry.second)
    {
      const Eigen::Vector3d local_position = cube_root * contact.pos;
      const Eigen::Vector3d local_normal = cube_root.linear() * contact.normal;
      ++target->total;
      target->mean_local_position += local_position;
      target->mean_abs_local_normal += local_normal.cwiseAbs();
      target->max_depth = std::max(target->max_depth, std::abs(contact.depth));

      const bool side_face = positive_y ?
        (local_position.y() >= hy - FACE_TOLERANCE_M && local_position.y() <= hy + FACE_TOLERANCE_M) :
        (local_position.y() <= -hy + FACE_TOLERANCE_M && local_position.y() >= -hy - FACE_TOLERANCE_M);
      const bool interior = std::abs(local_position.x()) <= hx - EDGE_MARGIN_M &&
        std::abs(local_position.z()) <= hz - EDGE_MARGIN_M;
      const Eigen::Vector3d abs_normal = local_normal.cwiseAbs();
      const bool normal_y = abs_normal.y() >= NORMAL_Y_MIN &&
        abs_normal.y() > abs_normal.x() && abs_normal.y() > abs_normal.z();
      const bool depth_ok = std::abs(contact.depth) <= MAX_CONTACT_DEPTH_M;
      if (side_face)
        ++target->side_face;
      if (interior)
        ++target->interior;
      if (normal_y)
        ++target->y_normal_dominant;
      if (side_face && interior && normal_y && depth_ok)
        ++target->qualified;
    }
  }
  for (auto* target : {&left, &right})
  {
    if (target->total > 0)
    {
      target->mean_local_position /= static_cast<double>(target->total);
      target->mean_abs_local_normal /= static_cast<double>(target->total);
    }
  }
}

Evaluation evaluate_candidate(
  planning_scene::PlanningScene& scene,
  const std::vector<PreparedSweepState>& states,
  const Fixture& fixture,
  const Eigen::Isometry3d& root_link7,
  const Candidate& candidate,
  const collision_detection::AllowedCollisionMatrix& fixture_acm,
  const collision_detection::CollisionRequest& request)
{
  scene.getWorldNonConst()->clearObjects();
  std::vector<Eigen::Isometry3d> poses;
  for (const auto& object : fixture.objects)
  {
    Eigen::Isometry3d link7_object = Eigen::Isometry3d::Identity();
    link7_object.translation() = object.offset_link7 + candidate.translation;
    poses.push_back(root_link7 * link7_object);
    scene.getWorldNonConst()->addToObject(object.name, object.shape, poses.back());
  }

  const std::set<std::string> allowed_cube_pairs = {
    "gripper_left_link|rubiks_cube", "gripper_right_link|rubiks_cube"
  };
  Evaluation result;
  result.first_dual_index = states.size();
  for (std::size_t index = 0; index < states.size(); ++index)
  {
    collision_detection::CollisionResult collision;
    scene.checkCollision(request, collision, *states[index].robot, fixture_acm);
    const auto pairs = contact_pairs(collision);
    const bool left_contact = pairs.count("gripper_left_link|rubiks_cube") != 0;
    const bool right_contact = pairs.count("gripper_right_link|rubiks_cube") != 0;
    const bool desired_contact = left_contact || right_contact;
    const bool dual_contact = left_contact && right_contact;
    bool forbidden_fixture = false;
    for (const auto& pair : pairs)
    {
      if (pair.find("rubiks_support") != std::string::npos)
        forbidden_fixture = true;
      if (pair.find("rubiks_cube") != std::string::npos && allowed_cube_pairs.count(pair) == 0)
        forbidden_fixture = true;
    }
    const bool forbidden = states[index].static_forbidden || forbidden_fixture;

    if (!forbidden && !desired_contact)
      result.open_clear = true;
    if (forbidden && !dual_contact)
    {
      result.forbidden_before_dual = true;
      result.decision = result.open_clear ?
        "BLOCKED_FORBIDDEN_COLLISION_BEFORE_FIRST_DUAL" :
        "BLOCKED_NO_COLLISION_FREE_OPENING_STATE";
      return result;
    }
    if (!forbidden && dual_contact)
    {
      result.first_dual_found = true;
      result.first_dual_index = index;
      result.first_dual_q = states[index].q;
      accumulate_quality(collision, poses[0], fixture.objects[0].size, result.left, result.right);
      const auto fraction = [](const FingerQuality& quality) {
        return quality.total == 0 ? 0.0 : static_cast<double>(quality.qualified) / static_cast<double>(quality.total);
      };
      result.quality_pass = result.left.total > 0 && result.right.total > 0 &&
        fraction(result.left) >= MIN_QUALITY_FRACTION && fraction(result.right) >= MIN_QUALITY_FRACTION;
      result.decision = result.quality_pass ?
        "OFFLINE_FIRST_DUAL_OPPOSING_SIDE_FACE_CONTACT_FOUND" :
        "BLOCKED_FIRST_DUAL_CONTACT_NOT_OPPOSING_SIDE_FACE_QUALITY";
      return result;
    }
  }
  result.decision = result.open_clear ?
    "BLOCKED_NO_DUAL_FINGER_CUBE_CONTACT_IN_LIMITS" :
    "BLOCKED_NO_COLLISION_FREE_OPENING_STATE";
  return result;
}

void write_quality(std::ostream& output, const FingerQuality& quality)
{
  const double fraction = quality.total == 0 ? 0.0 :
    static_cast<double>(quality.qualified) / static_cast<double>(quality.total);
  output << quality.total << '\t' << quality.qualified << '\t' << fraction << '\t'
         << quality.max_depth << '\t'
         << quality.mean_local_position.x() << '\t' << quality.mean_local_position.y() << '\t'
         << quality.mean_local_position.z() << '\t'
         << quality.mean_abs_local_normal.x() << '\t' << quality.mean_abs_local_normal.y() << '\t'
         << quality.mean_abs_local_normal.z();
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 8)
      throw std::runtime_error(
        "usage: opposing_side_contact_search URDF SRDF STATES FIXTURE CANDIDATES SCAN_OUTPUT COMPATIBLE_OUTPUT");

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
      throw std::runtime_error("required robot model identity missing");

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

    moveit::core::RobotState anchor(robot_model);
    anchor.setToDefaultValues();
    anchor.setJointGroupPositions(arm_group, states[fixture.anchor_index].arm);
    anchor.setVariablePosition("gripper_left_joint", upper);
    anchor.update();
    const Eigen::Isometry3d root_link7 = anchor.getGlobalLinkTransform("link7");

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

    const auto values = sweep_values(lower, upper);
    const auto prepared = prepare_sweep_states(
      robot_model, arm_group, states[fixture.anchor_index], values,
      scene, original_acm, ground_acm, ground_shape, request);

    std::vector<std::pair<Candidate, Evaluation>> evaluations;
    std::vector<Candidate> compatible;
    evaluations.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      Evaluation evaluation = evaluate_candidate(
        scene, prepared, fixture, root_link7, candidates[index], fixture_acm, request);
      if (evaluation.quality_pass)
        compatible.push_back(candidates[index]);
      evaluations.emplace_back(candidates[index], std::move(evaluation));
      if ((index + 1) % 50 == 0 || index + 1 == candidates.size())
        std::cout << "completed=" << (index + 1) << '/' << candidates.size()
                  << " quality_compatible=" << compatible.size() << '\n' << std::flush;
    }

    std::ofstream scan(argv[6]);
    if (!scan)
      throw std::runtime_error("cannot open scan output");
    scan << std::setprecision(17);
    scan << "META\t1\tXZ_OPPOSING_SIDE_CONTACT\t" << evaluations.size() << '\t'
         << compatible.size() << '\t' << FACE_TOLERANCE_M << '\t' << EDGE_MARGIN_M << '\t'
         << NORMAL_Y_MIN << '\t' << MAX_CONTACT_DEPTH_M << '\t' << MIN_QUALITY_FRACTION << '\n';
    for (const auto& item : evaluations)
    {
      const auto& candidate = item.first;
      const auto& evaluation = item.second;
      scan << "SCAN\t" << candidate.rank << '\t'
           << candidate.translation.x() << '\t' << candidate.translation.y() << '\t'
           << candidate.translation.z() << '\t' << candidate.distance << '\t'
           << candidate.dx << '\t' << candidate.dz << '\t' << evaluation.decision << '\t'
           << evaluation.first_dual_q << '\t';
      write_quality(scan, evaluation.left);
      scan << '\t';
      write_quality(scan, evaluation.right);
      scan << '\n';
    }

    std::ofstream accepted(argv[7]);
    if (!accepted)
      throw std::runtime_error("cannot open compatible output");
    accepted << std::setprecision(17);
    accepted << "META\t1\tXZ_OPPOSING_SIDE_CONTACT_COMPATIBLE\t" << compatible.size() << '\n';
    for (std::size_t index = 0; index < compatible.size(); ++index)
    {
      const auto& candidate = compatible[index];
      accepted << "CANDIDATE\t" << index << '\t'
               << candidate.translation.x() << '\t' << candidate.translation.y() << '\t'
               << candidate.translation.z() << '\t' << candidate.translation.norm() << '\t'
               << candidate.dx << "\t0\t" << candidate.dz << "\t0\t0\t0\n";
    }
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
