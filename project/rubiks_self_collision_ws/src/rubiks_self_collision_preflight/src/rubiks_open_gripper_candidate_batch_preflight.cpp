// Candidate-only full-path evaluator for the intended runtime order:
// fully-open gripper -> unchanged accepted arm trajectory -> close at anchor.
//
// Reuse the exact PR26/PR28 parsing, fixture placement and collision semantics,
// but intentionally omit the legacy zero-translation baseline assertion because
// this phase changes only the gripper state contract from passive-near-zero to
// fully open.
#define main rubiks_relevance_candidate_batch_preflight_legacy_main
#include "rubiks_relevance_candidate_batch_preflight.cpp"
#undef main

namespace
{
constexpr double EXPECTED_OPEN_GRIPPER_M = 0.019;
constexpr double OPEN_TOLERANCE_M = 1e-12;
}

int main(int argc, char** argv)
{
  try
  {
    if (argc != 7)
      throw std::runtime_error(
        "usage: open_gripper_batch URDF SRDF OPEN_STATES FIXTURE CANDIDATES OUTPUT");

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
    if (states.size() != 2802 || fixture.state_count != states.size() ||
        fixture.anchor_index >= states.size())
      throw std::runtime_error("open-state/fixture identity mismatch");
    for (std::size_t index = 0; index < states.size(); ++index)
    {
      if (std::abs(states[index].left_gripper - EXPECTED_OPEN_GRIPPER_M) > OPEN_TOLERANCE_M)
        throw std::runtime_error("state is not fully open at index " + std::to_string(index));
    }

    const auto left_joint = urdf_model->getJoint("gripper_left_joint");
    const auto right_joint = urdf_model->getJoint("gripper_right_joint");
    if (!left_joint || !right_joint || !left_joint->limits || !right_joint->mimic)
      throw std::runtime_error("gripper joint contract missing");
    if (std::abs(left_joint->limits->upper - EXPECTED_OPEN_GRIPPER_M) > OPEN_TOLERANCE_M ||
        right_joint->mimic->joint_name != "gripper_left_joint" ||
        std::abs(right_joint->mimic->multiplier - 1.0) > OPEN_TOLERANCE_M)
      throw std::runtime_error("fully-open/mimic contract mismatch");

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

    // prepare_states independently verifies bounds and self-collision for every
    // rewritten open-gripper state before any fixture candidate is evaluated.
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

    std::ofstream output(argv[6]);
    if (!output)
      throw std::runtime_error("cannot open output");
    output << std::setprecision(17);
    output << "META\t1\tOPEN_GRIPPER_PATH\t" << robot_model->getName() << '\t'
           << states.size() << '\t' << candidates.size() << '\t'
           << fixture.anchor_index << '\t' << EXPECTED_OPEN_GRIPPER_M << '\n';

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
