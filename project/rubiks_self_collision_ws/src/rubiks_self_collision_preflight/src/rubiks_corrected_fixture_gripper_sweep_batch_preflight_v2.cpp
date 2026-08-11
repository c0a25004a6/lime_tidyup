// Reuse the parsing/model helpers from the experimental v1 translation unit,
// but replace its main/evaluation with evidence-compatible semantics.
#define main rubiks_corrected_fixture_gripper_sweep_batch_preflight_v1_main
#include "rubiks_corrected_fixture_gripper_sweep_batch_preflight.cpp"
#undef main

namespace
{
Result evaluate_candidate_v2(
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
    if (forbidden && !result.dual_contact)
      result.forbidden_before_dual = true;

    // Exact decision-preserving early exits:
    // - once both open-clear and a prior forbidden state are known, the final
    //   PR27 decision must be forbidden-before-dual;
    // - a positive verdict is final only if no forbidden state preceded dual.
    if (result.open_clear && result.forbidden_before_dual)
    {
      result.decision = "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT";
      return result;
    }
    if (result.open_clear && result.dual_contact && !result.forbidden_before_dual)
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
        "usage: gripper_sweep_batch_v2 URDF SRDF STATES FIXTURE CANDIDATES SCAN COMPATIBLE");

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
      auto result = evaluate_candidate_v2(
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
    // Keep exact compatibility with the Python 3-D scan evidence schema.
    scan_output << "META\t1\tXYZ_REMAINING\t" << candidates.size() << '\t'
                << compatible_count << '\n';
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
                  << (result.forbidden_before_dual ? "true" : "false") << '\n';
      if (result.decision == "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND")
      {
        compatible_output << "CANDIDATE\t" << compatible_rank++ << '\t'
                          << candidate.translation.x() << '\t' << candidate.translation.y() << '\t'
                          << candidate.translation.z() << '\t' << candidate.translation.norm() << '\t'
                          << candidate.dx << '\t' << candidate.dy << '\t' << candidate.dz
                          << "\t0\t0\t0\n";
      }
    }
    std::cout << "total_sweep_state_checks=" << total_checks << '\n';
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
