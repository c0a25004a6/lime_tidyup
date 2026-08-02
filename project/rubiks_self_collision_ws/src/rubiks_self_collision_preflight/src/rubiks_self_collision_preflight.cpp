#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
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

struct PreviewState
{
  std::string label;
  std::vector<double> positions;
};

std::vector<PreviewState> read_states(const std::string& path, std::size_t expected_count)
{
  std::ifstream stream(path);
  if (!stream)
  {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<PreviewState> states;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
    {
      continue;
    }
    const auto fields = split(line, '\t');
    if (fields.size() != expected_count + 1)
    {
      throw std::runtime_error("invalid state field count for " + line);
    }
    PreviewState state;
    state.label = fields.front();
    for (std::size_t index = 1; index < fields.size(); ++index)
    {
      std::size_t consumed = 0;
      const double value = std::stod(fields[index], &consumed);
      if (consumed != fields[index].size())
      {
        throw std::runtime_error("invalid numeric state field " + fields[index]);
      }
      state.positions.push_back(value);
    }
    states.push_back(std::move(state));
  }
  return states;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 5)
    {
      throw std::runtime_error("usage: rubiks_self_collision_preflight URDF SRDF STATES_TSV OUTPUT_TSV");
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

    const std::vector<std::string> required_collision_links = {
      "link1", "link2", "link3", "link4", "link5", "link6", "link7",
      "gripper_left_link", "gripper_right_link"
    };
    std::vector<std::string> missing_geometry;
    for (const auto& name : required_collision_links)
    {
      const moveit::core::LinkModel* link = robot_model->getLinkModel(name);
      if (link == nullptr || link->getShapes().empty())
      {
        missing_geometry.push_back(name);
      }
    }

    planning_scene::PlanningScene scene(robot_model);
    bool adjacent_allowed = false;
    const bool adjacent_entry_present =
      scene.getAllowedCollisionMatrix().getEntry("link1", "link2", adjacent_allowed);

    const auto states = read_states(argv[3], expected_joints.size());
    if (states.size() != 7)
    {
      throw std::runtime_error("expected seed plus six preview states");
    }

    std::ofstream output(argv[4]);
    if (!output)
    {
      throw std::runtime_error("cannot open output TSV");
    }
    output << "META\t" << robot_model->getName() << "\tarm\t" << join(active_joints, ",") << "\t"
           << join(missing_geometry, ",") << "\t" << (adjacent_entry_present ? "true" : "false") << "\t"
           << (adjacent_allowed ? "true" : "false") << "\n";

    collision_detection::CollisionRequest request;
    request.group_name = "arm";
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;

    output << std::setprecision(17);
    for (const auto& preview : states)
    {
      moveit::core::RobotState state(robot_model);
      state.setToDefaultValues();
      state.setJointGroupPositions(arm_group, preview.positions);
      state.update();

      const bool bounds_ok = state.satisfiesBounds(arm_group, 0.0);
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state, scene.getAllowedCollisionMatrix());

      std::set<std::string> contact_pairs;
      for (const auto& entry : result.contacts)
      {
        std::string first = entry.first.first;
        std::string second = entry.first.second;
        if (second < first)
        {
          std::swap(first, second);
        }
        contact_pairs.insert(first + "|" + second);
      }
      output << "STATE\t" << preview.label << "\t" << (bounds_ok ? "true" : "false") << "\t"
             << (result.collision ? "true" : "false") << "\t" << contact_pairs.size() << "\t"
             << join_set(contact_pairs, ",") << "\n";
    }
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
