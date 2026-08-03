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
  {
    throw std::runtime_error("cannot open " + path);
  }
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
  {
    values.push_back(value);
  }
  return values;
}

std::string join(const std::vector<std::string>& values, const std::string& delimiter)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index)
  {
    if (index != 0)
    {
      stream << delimiter;
    }
    stream << values[index];
  }
  return stream.str();
}

std::string join_set(const std::set<std::string>& values, const std::string& delimiter)
{
  return join(std::vector<std::string>(values.begin(), values.end()), delimiter);
}

struct AuditState
{
  std::string label;
  Eigen::Vector3d root_position_world;
  Eigen::Quaterniond root_orientation_world;
  std::vector<double> arm_positions;
};

std::vector<AuditState> read_states(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
  {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<AuditState> states;
  std::set<std::string> labels;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
    {
      continue;
    }
    const auto fields = split(line, '\t');
    if (fields.size() != 14)
    {
      throw std::runtime_error("invalid ground-state field count for " + line);
    }
    AuditState state;
    state.label = fields[0];
    if (state.label.empty() || !labels.insert(state.label).second)
    {
      throw std::runtime_error("empty or duplicate state label: " + state.label);
    }
    std::vector<double> values;
    for (std::size_t index = 1; index < fields.size(); ++index)
    {
      std::size_t consumed = 0;
      const double value = std::stod(fields[index], &consumed);
      if (consumed != fields[index].size() || !std::isfinite(value))
      {
        throw std::runtime_error("invalid numeric state field " + fields[index]);
      }
      values.push_back(value);
    }
    state.root_position_world = Eigen::Vector3d(values[0], values[1], values[2]);
    state.root_orientation_world = Eigen::Quaterniond(values[6], values[3], values[4], values[5]);
    if (state.root_orientation_world.norm() < 1e-12)
    {
      throw std::runtime_error("zero root quaternion for " + state.label);
    }
    state.root_orientation_world.normalize();
    state.arm_positions.assign(values.begin() + 7, values.end());
    states.push_back(std::move(state));
  }
  if (states.empty())
  {
    throw std::runtime_error("ground-state matrix is empty");
  }
  return states;
}

std::set<std::string> contact_pairs(const collision_detection::CollisionResult& result)
{
  std::set<std::string> pairs;
  for (const auto& entry : result.contacts)
  {
    std::string first = entry.first.first;
    std::string second = entry.first.second;
    if (second < first)
    {
      std::swap(first, second);
    }
    pairs.insert(first + "|" + second);
  }
  return pairs;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 5)
    {
      throw std::runtime_error(
        "usage: rubiks_ground_plane_clearance_preflight URDF SRDF STATES_TSV OUTPUT_TSV");
    }

    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
    {
      throw std::runtime_error("URDF parsing failed");
    }
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
    {
      throw std::runtime_error("SRDF parsing failed");
    }
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    const moveit::core::JointModelGroup* arm_group = robot_model->getJointModelGroup("arm");
    if (arm_group == nullptr)
    {
      throw std::runtime_error("SRDF arm group is missing");
    }

    const std::vector<std::string> expected_joints = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    const auto active_joints = arm_group->getActiveJointModelNames();
    if (active_joints != expected_joints)
    {
      throw std::runtime_error("active arm joint order mismatch: " + join(active_joints, ","));
    }
    if (robot_model->getRootLinkName() != "base_footprint")
    {
      throw std::runtime_error("unexpected MoveIt root link: " + robot_model->getRootLinkName());
    }

    const std::vector<std::string> audited_links = {
      "link1", "link2", "link3", "link4", "link5", "link6", "link7"
    };
    const std::set<std::string> audited_set(audited_links.begin(), audited_links.end());
    std::vector<std::string> missing_geometry;
    for (const auto& name : audited_links)
    {
      const moveit::core::LinkModel* link = robot_model->getLinkModel(name);
      if (link == nullptr || link->getShapes().empty())
      {
        missing_geometry.push_back(name);
      }
    }

    planning_scene::PlanningScene scene(robot_model);
    const auto original_acm = scene.getAllowedCollisionMatrix();
    auto ground_acm = original_acm;
    const auto all_links = robot_model->getLinkModelNames();
    for (const auto& first : all_links)
    {
      for (const auto& second : all_links)
      {
        ground_acm.setEntry(first, second, true);
      }
    }
    std::size_t allowed_ground_link_count = 0;
    for (const auto& link : all_links)
    {
      const bool allowed = audited_set.count(link) == 0;
      ground_acm.setEntry("ground_plane", link, allowed);
      if (allowed)
      {
        ++allowed_ground_link_count;
      }
    }

    const auto states = read_states(argv[3]);
    std::ofstream output(argv[4]);
    if (!output)
    {
      throw std::runtime_error("cannot open output TSV");
    }
    output << "META\t" << robot_model->getName() << "\t" << robot_model->getRootLinkName() << "\t"
           << join(active_joints, ",") << "\t" << join(audited_links, ",") << "\t"
           << join(missing_geometry, ",") << "\t" << srdf_model->getDisabledCollisionPairs().size() << "\t"
           << allowed_ground_link_count << "\n";
    output.flush();

    collision_detection::CollisionRequest self_request;
    self_request.contacts = true;
    self_request.max_contacts = 1000;
    self_request.max_contacts_per_pair = 100;

    collision_detection::CollisionRequest ground_request;
    ground_request.contacts = true;
    // FCL's Humble plane-distance path is not needed for this contact-only gate.
    // Requesting distance for an infinite plane can crash before producing evidence.
    ground_request.distance = false;
    ground_request.max_contacts = 1000;
    ground_request.max_contacts_per_pair = 100;

    output << std::setprecision(17);
    std::size_t state_index = 0;
    for (const auto& audit : states)
    {
      moveit::core::RobotState state(robot_model);
      state.setToDefaultValues();
      state.setJointGroupPositions(arm_group, audit.arm_positions);
      state.update();
      const bool bounds_ok = state.satisfiesBounds(arm_group, 0.0);

      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(self_request, self_result, state, original_acm);
      const auto self_pairs = contact_pairs(self_result);

      const Eigen::Matrix3d root_rotation_world = audit.root_orientation_world.toRotationMatrix();
      const Eigen::Vector3d normal_root = root_rotation_world.transpose() * Eigen::Vector3d::UnitZ();
      const double offset_root = audit.root_position_world.z();
      shapes::ShapeConstPtr plane(
        new shapes::Plane(normal_root.x(), normal_root.y(), normal_root.z(), offset_root));
      scene.getWorldNonConst()->clearObjects();
      scene.getWorldNonConst()->addToObject(
        "ground_plane", plane, Eigen::Isometry3d::Identity());

      collision_detection::CollisionResult ground_result;
      scene.checkCollision(ground_request, ground_result, state, ground_acm);
      const auto ground_pairs = contact_pairs(ground_result);

      output << "STATE\t" << audit.label << "\t" << (bounds_ok ? "true" : "false") << "\t"
             << (self_result.collision ? "true" : "false") << "\t"
             << (ground_result.collision ? "true" : "false") << "\t"
             << ground_pairs.size() << "\t" << join_set(ground_pairs, ",") << "\t"
             << self_pairs.size() << "\t" << join_set(self_pairs, ",") << "\tNA\n";
      ++state_index;
      if (state_index == 1 || state_index % 100 == 0)
      {
        output.flush();
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
