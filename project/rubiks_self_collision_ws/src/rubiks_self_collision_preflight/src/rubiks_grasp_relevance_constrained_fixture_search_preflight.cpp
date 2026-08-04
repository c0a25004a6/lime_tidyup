#define main rubiks_unconstrained_alignment_search_main
#include "rubiks_fixture_alignment_search_preflight_v2.cpp"
#undef main

namespace
{
constexpr std::size_t EXPECTED_COARSE_TOTAL = 267761;
constexpr std::size_t EXPECTED_COARSE_RELEVANT = 61511;
constexpr double REACH_TOLERANCE = 1e-12;

struct ReachContract
{
  std::size_t state_count = 0;
  std::size_t anchor_index = 0;
  std::size_t candidate_start_index = 0;
  std::string reference_link;
  Eigen::Vector3d cube_size = Eigen::Vector3d::Zero();
  Eigen::Vector3d cube_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_size = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d left_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d left_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_max = Eigen::Vector3d::Zero();
  double gripper_lower = 0.0;
  double gripper_upper = 0.0;
  double coarse_step = 0.0;
  double coarse_radius = 0.0;
  std::size_t coarse_count = 0;
  std::size_t coarse_relevant_count = 0;
  int coarse_min_z_step = 0;
  int coarse_max_z_step = 0;
  double fine_step = 0.0;
  double fine_radius = 0.0;
  std::size_t fine_count = 0;
  Eigen::Vector3d blocked_translation = Eigen::Vector3d::Zero();
  std::string external_source;
};

struct ReachEvaluation
{
  bool relevant = false;
  double left_x_overlap = 0.0;
  double left_z_overlap = 0.0;
  double right_x_overlap = 0.0;
  double right_z_overlap = 0.0;
  double left_low = 0.0;
  double left_high = 0.0;
  double right_low = 0.0;
  double right_high = 0.0;
  double common_low = 0.0;
  double common_high = 0.0;
  bool common_exists = false;
};

bool near_value(double first, double second)
{
  return std::abs(first - second) <= 1e-15;
}

bool near_vector(const Eigen::Vector3d& first, const Eigen::Vector3d& second)
{
  return (first - second).cwiseAbs().maxCoeff() <= 1e-15;
}

ReachContract read_reach_contract(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  ReachContract contract;
  bool meta = false;
  bool cube = false;
  bool support = false;
  bool left = false;
  bool right = false;
  bool gripper = false;
  bool coarse = false;
  bool fine = false;
  bool blocked = false;
  bool external = false;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields.empty())
      throw std::runtime_error("empty reach contract row");
    if (fields[0] == "META")
    {
      if (fields.size() != 7 || fields[1] != "1")
        throw std::runtime_error("invalid reach META row");
      contract.state_count = parse_integer(fields[2]);
      contract.anchor_index = parse_integer(fields[3]);
      contract.candidate_start_index = parse_integer(fields[4]);
      contract.reference_link = fields[5];
      if (!fields[6].empty())
        throw std::runtime_error("unexpected reach META tail");
      meta = true;
    }
    else if (fields[0] == "CUBE" || fields[0] == "SUPPORT")
    {
      if (fields.size() != 7)
        throw std::runtime_error("invalid reach fixture row");
      const Eigen::Vector3d size(
        parse_number(fields[1]), parse_number(fields[2]), parse_number(fields[3]));
      const Eigen::Vector3d center(
        parse_number(fields[4]), parse_number(fields[5]), parse_number(fields[6]));
      if ((size.array() <= 0.0).any())
        throw std::runtime_error("nonpositive reach fixture size");
      if (fields[0] == "CUBE")
      {
        contract.cube_size = size;
        contract.cube_center = center;
        cube = true;
      }
      else
      {
        contract.support_size = size;
        contract.support_center = center;
        support = true;
      }
    }
    else if (fields[0] == "LEFT_ZERO" || fields[0] == "RIGHT_ZERO")
    {
      if (fields.size() != 7)
        throw std::runtime_error("invalid reach AABB row");
      const Eigen::Vector3d minimum(
        parse_number(fields[1]), parse_number(fields[2]), parse_number(fields[3]));
      const Eigen::Vector3d maximum(
        parse_number(fields[4]), parse_number(fields[5]), parse_number(fields[6]));
      if ((maximum.array() <= minimum.array()).any())
        throw std::runtime_error("nonpositive reach AABB extent");
      if (fields[0] == "LEFT_ZERO")
      {
        contract.left_min = minimum;
        contract.left_max = maximum;
        left = true;
      }
      else
      {
        contract.right_min = minimum;
        contract.right_max = maximum;
        right = true;
      }
    }
    else if (fields[0] == "GRIPPER")
    {
      if (fields.size() != 3)
        throw std::runtime_error("invalid reach GRIPPER row");
      contract.gripper_lower = parse_number(fields[1]);
      contract.gripper_upper = parse_number(fields[2]);
      if (!(contract.gripper_lower < contract.gripper_upper))
        throw std::runtime_error("invalid gripper reach limits");
      gripper = true;
    }
    else if (fields[0] == "COARSE")
    {
      if (fields.size() != 7)
        throw std::runtime_error("invalid reach COARSE row");
      contract.coarse_step = parse_number(fields[1]);
      contract.coarse_radius = parse_number(fields[2]);
      contract.coarse_count = parse_integer(fields[3]);
      contract.coarse_relevant_count = parse_integer(fields[4]);
      contract.coarse_min_z_step = static_cast<int>(std::stoll(fields[5]));
      contract.coarse_max_z_step = static_cast<int>(std::stoll(fields[6]));
      coarse = true;
    }
    else if (fields[0] == "FINE")
    {
      if (fields.size() != 4)
        throw std::runtime_error("invalid reach FINE row");
      contract.fine_step = parse_number(fields[1]);
      contract.fine_radius = parse_number(fields[2]);
      contract.fine_count = parse_integer(fields[3]);
      fine = true;
    }
    else if (fields[0] == "PR24_BLOCKED")
    {
      if (fields.size() != 4)
        throw std::runtime_error("invalid reach PR24_BLOCKED row");
      contract.blocked_translation = Eigen::Vector3d(
        parse_number(fields[1]), parse_number(fields[2]), parse_number(fields[3]));
      blocked = true;
    }
    else if (fields[0] == "EXTERNAL")
    {
      if (fields.size() != 2)
        throw std::runtime_error("invalid reach EXTERNAL row");
      contract.external_source = fields[1];
      external = true;
    }
    else
      throw std::runtime_error("unknown reach contract row: " + fields[0]);
  }
  if (!meta || !cube || !support || !left || !right || !gripper ||
      !coarse || !fine || !blocked || !external)
    throw std::runtime_error("reach contract is incomplete");
  if (contract.state_count != 2802 || contract.anchor_index != 1541 ||
      contract.candidate_start_index != 1043 || contract.reference_link != "link7")
    throw std::runtime_error("reach identity mismatch");
  if (!near_value(contract.coarse_step, COARSE_STEP_M) ||
      !near_value(contract.coarse_radius, COARSE_RADIUS_STEPS * COARSE_STEP_M) ||
      contract.coarse_count != EXPECTED_COARSE_TOTAL ||
      contract.coarse_relevant_count != EXPECTED_COARSE_RELEVANT ||
      contract.coarse_min_z_step != -40 || contract.coarse_max_z_step != 22)
    throw std::runtime_error("coarse reach search contract mismatch");
  if (!near_value(contract.fine_step, FINE_STEP_M) ||
      !near_value(contract.fine_radius, FINE_RADIUS_STEPS * FINE_STEP_M) ||
      contract.fine_count != 4913)
    throw std::runtime_error("fine reach search contract mismatch");
  if (!near_vector(contract.blocked_translation, Eigen::Vector3d(0.0, 0.0, 0.04575)))
    throw std::runtime_error("PR24 blocked translation mismatch");
  if (contract.external_source != "3363b27aae6d37598dc142cf4dddb342fef3047f")
    throw std::runtime_error("external reach source mismatch");
  return contract;
}

double positive_overlap(double first_min, double first_max, double second_min, double second_max)
{
  return std::max(0.0, std::min(first_max, second_max) - std::max(first_min, second_min));
}

ReachEvaluation evaluate_reach(
  const ReachContract& contract,
  const Eigen::Vector3d& delta)
{
  const Eigen::Vector3d center = contract.cube_center + delta;
  const Eigen::Vector3d half = contract.cube_size * 0.5;
  const Eigen::Vector3d cube_min = center - half;
  const Eigen::Vector3d cube_max = center + half;
  ReachEvaluation evaluation;
  evaluation.left_x_overlap = positive_overlap(
    contract.left_min.x(), contract.left_max.x(), cube_min.x(), cube_max.x());
  evaluation.left_z_overlap = positive_overlap(
    contract.left_min.z(), contract.left_max.z(), cube_min.z(), cube_max.z());
  evaluation.right_x_overlap = positive_overlap(
    contract.right_min.x(), contract.right_max.x(), cube_min.x(), cube_max.x());
  evaluation.right_z_overlap = positive_overlap(
    contract.right_min.z(), contract.right_max.z(), cube_min.z(), cube_max.z());
  evaluation.left_low = cube_max.y() - contract.left_max.y();
  evaluation.left_high = cube_max.y() - contract.left_min.y();
  evaluation.right_low = contract.right_min.y() - cube_min.y();
  evaluation.right_high = contract.right_max.y() - cube_min.y();
  evaluation.common_low = std::max({
    contract.gripper_lower, evaluation.left_low, evaluation.right_low
  });
  evaluation.common_high = std::min({
    contract.gripper_upper, evaluation.left_high, evaluation.right_high
  });
  evaluation.common_exists = evaluation.common_low <= evaluation.common_high + REACH_TOLERANCE;
  evaluation.relevant =
    evaluation.left_x_overlap > 0.0 && evaluation.left_z_overlap > 0.0 &&
    evaluation.right_x_overlap > 0.0 && evaluation.right_z_overlap > 0.0 &&
    evaluation.common_exists;
  return evaluation;
}

void write_reach(std::ofstream& output, const ReachEvaluation& reach)
{
  output << "REACH\t" << (reach.relevant ? "true" : "false") << '\t'
         << reach.left_x_overlap << '\t' << reach.left_z_overlap << '\t'
         << reach.right_x_overlap << '\t' << reach.right_z_overlap << '\t'
         << reach.left_low << '\t' << reach.left_high << '\t'
         << reach.right_low << '\t' << reach.right_high << '\t'
         << reach.common_low << '\t' << reach.common_high << '\t'
         << (reach.common_exists ? "true" : "false") << '\n';
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 7)
      throw std::runtime_error(
        "usage: grasp_relevance_search URDF SRDF STATES FIXTURE REACH_CONTRACT OUTPUT");

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
    const ReachContract reach_contract = read_reach_contract(argv[5]);
    if (fixture.state_count != states.size() || fixture.anchor_index >= states.size() ||
        fixture.candidate_start_index > fixture.anchor_index)
      throw std::runtime_error("fixture/state count or anchor mismatch");
    if (reach_contract.state_count != fixture.state_count ||
        reach_contract.anchor_index != fixture.anchor_index ||
        reach_contract.candidate_start_index != fixture.candidate_start_index ||
        reach_contract.reference_link != fixture.reference_link)
      throw std::runtime_error("fixture/reach identity mismatch");
    if (!near_vector(fixture.objects[0].size, reach_contract.cube_size) ||
        !near_vector(fixture.objects[0].offset_link7, reach_contract.cube_center) ||
        !near_vector(fixture.objects[1].size, reach_contract.support_size) ||
        !near_vector(fixture.objects[1].offset_link7, reach_contract.support_center))
      throw std::runtime_error("fixture/reach geometry mismatch");

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

    const auto prepared = prepare_states(
      states, fixture, robot_model, arm_group, scene, original_acm);
    const Eigen::Isometry3d root_link7_anchor =
      prepared[fixture.anchor_index].robot->getGlobalLinkTransform("link7");
    const Eigen::Isometry3d world_link7_anchor =
      states[fixture.anchor_index].world_root * root_link7_anchor;

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.distance = false;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    const ReachEvaluation zero_reach = evaluate_reach(
      reach_contract, Eigen::Vector3d::Zero());
    if (!zero_reach.relevant)
      throw std::runtime_error("zero translation unexpectedly fails grasp relevance");
    const Evaluation baseline = evaluate_translation(
      scene, prepared, fixture, world_link7_anchor,
      Eigen::Vector3d::Zero(), fixture_acm, request);
    const std::set<std::string> expected_baseline_pairs = {
      "gripper_left_link|rubiks_cube",
      "gripper_left_link|rubiks_support",
      "gripper_right_link|rubiks_cube",
      "gripper_right_link|rubiks_support"
    };
    if (baseline.clear || baseline.first_collision_index != 0 ||
        !baseline.cube_collision || !baseline.support_collision ||
        baseline.pairs != expected_baseline_pairs)
      throw std::runtime_error("baseline did not reproduce accepted fixture blocker");
    const ReachEvaluation pr24_blocked_reach = evaluate_reach(
      reach_contract, reach_contract.blocked_translation);
    if (pr24_blocked_reach.relevant ||
        pr24_blocked_reach.left_z_overlap != 0.0 ||
        pr24_blocked_reach.right_z_overlap != 0.0 ||
        !pr24_blocked_reach.common_exists)
      throw std::runtime_error("PR24 grasp-relevance blocker was not reproduced");

    const auto coarse_all = coarse_candidates();
    if (coarse_all.size() != EXPECTED_COARSE_TOTAL)
      throw std::runtime_error("coarse candidate total mismatch");
    std::vector<CoarseCandidate> coarse_relevant;
    coarse_relevant.reserve(EXPECTED_COARSE_RELEVANT);
    int minimum_relevant_z = COARSE_RADIUS_STEPS;
    int maximum_relevant_z = -COARSE_RADIUS_STEPS;
    for (const auto& candidate : coarse_all)
    {
      if (!evaluate_reach(reach_contract, coarse_translation(candidate)).relevant)
        continue;
      coarse_relevant.push_back(candidate);
      minimum_relevant_z = std::min(minimum_relevant_z, candidate.z);
      maximum_relevant_z = std::max(maximum_relevant_z, candidate.z);
    }
    if (coarse_relevant.size() != EXPECTED_COARSE_RELEVANT ||
        minimum_relevant_z != -40 || maximum_relevant_z != 22)
      throw std::runtime_error("coarse grasp-relevance filter mismatch");

    SearchStats coarse_stats;
    bool coarse_found = false;
    CoarseCandidate coarse_selected;
    Eigen::Vector3d coarse_delta = Eigen::Vector3d::Zero();
    for (const auto& candidate : coarse_relevant)
    {
      const Eigen::Vector3d delta = coarse_translation(candidate);
      const Evaluation evaluation = evaluate_translation(
        scene, prepared, fixture, world_link7_anchor, delta, fixture_acm, request);
      record_evaluation(coarse_stats, evaluation);
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
    std::size_t fine_total_count = 0;
    std::size_t fine_relevant_count = 0;
    if (coarse_found)
    {
      const auto fine_all = fine_candidates(coarse_selected);
      fine_total_count = fine_all.size();
      if (fine_total_count != reach_contract.fine_count)
        throw std::runtime_error("fine candidate total mismatch");
      for (const auto& candidate : fine_all)
      {
        const Eigen::Vector3d delta = fine_translation(candidate);
        if (!evaluate_reach(reach_contract, delta).relevant)
          continue;
        ++fine_relevant_count;
        const Evaluation evaluation = evaluate_translation(
          scene, prepared, fixture, world_link7_anchor, delta, fixture_acm, request);
        record_evaluation(fine_stats, evaluation);
        if (evaluation.clear)
        {
          fine_found = true;
          selected_delta = delta;
          break;
        }
      }
      if (!fine_found)
        throw std::runtime_error("coarse relevant clear candidate was not retained by fine grid");
    }

    const bool selected_found = coarse_found && fine_found;
    const ReachEvaluation selected_reach = evaluate_reach(
      reach_contract, selected_delta);
    if (selected_found && !selected_reach.relevant)
      throw std::runtime_error("selected candidate lost grasp relevance");
    const std::string decision = selected_found ?
      "FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_FOUND" :
      "NO_FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_WITHIN_COARSE_RADIUS";

    std::ofstream output(argv[6]);
    if (!output)
      throw std::runtime_error("cannot open output TSV");
    output << std::setprecision(17);
    output << "META\t1\t" << robot_model->getName() << '\t' << robot_model->getRootLinkName()
           << '\t' << states.size() << '\t' << fixture.anchor_index << '\t'
           << fixture.candidate_start_index << '\t' << fixture.reference_link << '\n';
    output << "BASELINE\tfalse\t" << baseline.first_collision_index << '\t'
           << baseline.first_collision_phase << "\ttrue\ttrue\t"
           << baseline.states_checked << '\t' << join_set(baseline.pairs) << '\n';
    output << "FILTER\tCOARSE\t" << coarse_all.size() << '\t'
           << coarse_relevant.size() << '\t'
           << (coarse_all.size() - coarse_relevant.size()) << '\t'
           << minimum_relevant_z << '\t' << maximum_relevant_z << '\n';
    write_search_stats(
      output, "COARSE", COARSE_STEP_M,
      COARSE_RADIUS_STEPS * COARSE_STEP_M, coarse_relevant.size(),
      coarse_stats, coarse_found, coarse_delta);
    output << "FILTER\tFINE\t" << fine_total_count << '\t'
           << fine_relevant_count << '\t'
           << (fine_total_count - fine_relevant_count) << "\t0\t0\n";
    write_search_stats(
      output, "FINE", FINE_STEP_M,
      FINE_RADIUS_STEPS * FINE_STEP_M, fine_relevant_count,
      fine_stats, fine_found, selected_delta);
    output << "SELECT\t" << (selected_found ? "true" : "false") << '\t'
           << selected_delta.x() << '\t' << selected_delta.y() << '\t'
           << selected_delta.z() << '\t' << selected_delta.norm() << '\t'
           << decision << '\n';
    write_reach(output, selected_reach);
    output.flush();

    if (!selected_found)
      return 0;

    const auto selected_world_objects = object_world_poses(
      fixture, world_link7_anchor, selected_delta);
    collision_detection::CollisionRequest self_request = request;
    for (std::size_t index = 0; index < prepared.size(); ++index)
    {
      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(
        self_request, self_result, *prepared[index].robot, original_acm);
      const auto self_pairs = contact_pairs(self_result);
      const auto cube_result = check_single_object(
        scene, *prepared[index].robot, fixture_acm, fixture.objects[0],
        prepared[index].root_world * selected_world_objects[0], request);
      const auto support_result = check_single_object(
        scene, *prepared[index].robot, fixture_acm, fixture.objects[1],
        prepared[index].root_world * selected_world_objects[1], request);
      const auto cube_pairs = contact_pairs(cube_result);
      const auto support_pairs = contact_pairs(support_result);
      std::set<std::string> fixture_pairs = cube_pairs;
      fixture_pairs.insert(support_pairs.begin(), support_pairs.end());
      output << "STATE\t" << prepared[index].label << '\t' << prepared[index].phase
             << "\ttrue\t" << (self_result.collision ? "true" : "false") << '\t'
             << join_set(self_pairs) << '\t'
             << (cube_result.collision ? "true" : "false") << '\t'
             << (support_result.collision ? "true" : "false") << '\t'
             << fixture_pairs.size() << '\t' << join_set(fixture_pairs) << '\n';
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
