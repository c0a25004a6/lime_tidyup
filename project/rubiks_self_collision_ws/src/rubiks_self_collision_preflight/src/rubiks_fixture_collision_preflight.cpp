#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <array>
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
constexpr std::size_t kArmJointCount = 6;
const std::array<std::string, kArmJointCount> kArmJoints = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};

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
  std::vector<std::string> result;
  std::istringstream stream(text);
  std::string field;
  while (std::getline(stream, field, delimiter))
  {
    result.push_back(field);
  }
  return result;
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

double parse_number(const std::string& text)
{
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
  {
    throw std::runtime_error("invalid finite numeric field: " + text);
  }
  return value;
}

struct ObjectSpec
{
  std::string id;
  Eigen::Vector3d dimensions;
  Eigen::Vector3d relative_translation;
};

struct StateSpec
{
  std::string label;
  std::array<double, kArmJointCount> arm;
  double gripper_left;
};

std::vector<ObjectSpec> read_objects(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
  {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<ObjectSpec> objects;
  std::set<std::string> ids;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
    {
      continue;
    }
    const auto fields = split(line, '\t');
    if (fields.size() != 7)
    {
      throw std::runtime_error("invalid object row: " + line);
    }
    if (fields[0].empty() || !ids.insert(fields[0]).second)
    {
      throw std::runtime_error("empty or duplicate object ID: " + fields[0]);
    }
    ObjectSpec object;
    object.id = fields[0];
    object.dimensions = Eigen::Vector3d(
      parse_number(fields[1]), parse_number(fields[2]), parse_number(fields[3]));
    object.relative_translation = Eigen::Vector3d(
      parse_number(fields[4]), parse_number(fields[5]), parse_number(fields[6]));
    if ((object.dimensions.array() <= 0.0).any())
    {
      throw std::runtime_error("object dimensions must be positive: " + object.id);
    }
    objects.push_back(std::move(object));
  }
  if (objects.size() != 2)
  {
    throw std::runtime_error("expected exactly support and cube objects");
  }
  return objects;
}

std::vector<StateSpec> read_states(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
  {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<StateSpec> states;
  std::set<std::string> labels;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
    {
      continue;
    }
    const auto fields = split(line, '\t');
    if (fields.size() != 8)
    {
      throw std::runtime_error("invalid state row: " + line);
    }
    if (fields[0].empty() || !labels.insert(fields[0]).second)
    {
      throw std::runtime_error("empty or duplicate state label: " + fields[0]);
    }
    StateSpec state;
    state.label = fields[0];
    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
      state.arm[index] = parse_number(fields[index + 1]);
    }
    state.gripper_left = parse_number(fields[7]);
    states.push_back(std::move(state));
  }
  if (states.empty() || states.front().label != "measured_candidate")
  {
    throw std::runtime_error("state matrix must begin with measured_candidate");
  }
  return states;
}

void set_state(moveit::core::RobotState& state, const StateSpec& values)
{
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < kArmJointCount; ++index)
  {
    positions[kArmJoints[index]] = values.arm[index];
  }
  positions["gripper_left_joint"] = values.gripper_left;
  state.setToDefaultValues();
  state.setVariablePositions(positions);
  state.update();
}

std::string matrix_fields(const Eigen::Isometry3d& transform)
{
  std::ostringstream stream;
  stream << std::setprecision(17);
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      stream << '\t' << transform.matrix()(row, column);
    }
  }
  return stream.str();
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 6)
    {
      throw std::runtime_error(
        "usage: rubiks_fixture_collision_preflight URDF SRDF OBJECTS_TSV STATES_TSV OUTPUT_TSV");
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
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    const auto* gripper_group = robot_model->getJointModelGroup("gripper");
    if (arm_group == nullptr || gripper_group == nullptr)
    {
      throw std::runtime_error("SRDF arm or gripper group is missing");
    }
    const auto active_arm = arm_group->getActiveJointModelNames();
    if (active_arm != std::vector<std::string>(kArmJoints.begin(), kArmJoints.end()))
    {
      throw std::runtime_error("active arm joint order mismatch");
    }

    const std::vector<std::string> required_collision_links = {
      "link1", "link2", "link3", "link4", "link5", "link6", "link7",
      "gripper_left_link", "gripper_right_link"
    };
    std::vector<std::string> missing_geometry;
    for (const auto& name : required_collision_links)
    {
      const auto* link = robot_model->getLinkModel(name);
      if (link == nullptr || link->getShapes().empty())
      {
        missing_geometry.push_back(name);
      }
    }

    const auto objects = read_objects(argv[3]);
    const auto states = read_states(argv[4]);
    planning_scene::PlanningScene scene(robot_model);
    moveit::core::RobotState candidate(robot_model);
    set_state(candidate, states.front());
    const Eigen::Isometry3d link7_transform = candidate.getGlobalLinkTransform("link7");

    std::map<std::string, Eigen::Isometry3d> world_transforms;
    std::set<std::string> object_ids;
    for (const auto& object : objects)
    {
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.translation() = object.relative_translation;
      const Eigen::Isometry3d world_pose = link7_transform * relative;
      shapes::ShapePtr shape = std::make_shared<shapes::Box>(
        object.dimensions.x(), object.dimensions.y(), object.dimensions.z());
      scene.getWorldNonConst()->addToObject(object.id, shape, world_pose);
      world_transforms[object.id] = world_pose;
      object_ids.insert(object.id);
    }

    std::ofstream output(argv[5]);
    if (!output)
    {
      throw std::runtime_error("cannot open output TSV");
    }
    output << std::setprecision(17);
    output << "META\t" << robot_model->getName() << "\t" << states.size() << "\t"
           << missing_geometry.size() << "\t" << candidate.getVariablePosition("gripper_left_joint") << "\t"
           << candidate.getVariablePosition("gripper_right_joint") << "\n";
    for (const auto& object : objects)
    {
      output << "OBJECT\t" << object.id << '\t' << object.dimensions.x() << '\t' << object.dimensions.y() << '\t'
             << object.dimensions.z() << '\t' << object.relative_translation.x() << '\t'
             << object.relative_translation.y() << '\t' << object.relative_translation.z()
             << matrix_fields(world_transforms.at(object.id)) << "\n";
    }

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    for (const auto& values : states)
    {
      moveit::core::RobotState state(robot_model);
      set_state(state, values);
      const bool arm_bounds = state.satisfiesBounds(arm_group, 0.0);
      const bool gripper_bounds = state.satisfiesBounds(gripper_group, 0.0);

      collision_detection::CollisionResult result;
      scene.checkCollision(request, result, state, scene.getAllowedCollisionMatrix());
      std::set<std::string> self_pairs;
      std::set<std::string> world_pairs;
      for (const auto& entry : result.contacts)
      {
        std::string first = entry.first.first;
        std::string second = entry.first.second;
        if (second < first)
        {
          std::swap(first, second);
        }
        const std::string pair = first + "|" + second;
        if (object_ids.count(first) != 0 || object_ids.count(second) != 0)
        {
          world_pairs.insert(pair);
        }
        else
        {
          self_pairs.insert(pair);
        }
      }
      output << "STATE\t" << values.label << '\t' << (arm_bounds ? "true" : "false") << '\t'
             << (gripper_bounds ? "true" : "false") << '\t' << (result.collision ? "true" : "false") << '\t'
             << self_pairs.size() << '\t' << world_pairs.size() << '\t' << join_set(self_pairs, ",") << '\t'
             << join_set(world_pairs, ",") << "\n";
    }
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
