#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr double TOLERANCE = 1e-12;

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
  const unsigned long long value = std::stoull(text, &consumed);
  if (consumed != text.size())
    throw std::runtime_error("invalid integer: " + text);
  return static_cast<std::size_t>(value);
}

struct Aabb
{
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
};

struct Contract
{
  std::size_t state_count = 0;
  std::size_t anchor_index = 0;
  std::string reference_link;
  Eigen::Vector3d cube_size = Eigen::Vector3d::Zero();
  Eigen::Vector3d cube_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_size = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double lower = 0.0;
  double zero = 0.0;
  double passive = 0.0;
  double upper = 0.0;
  std::string anchor_label;
  std::vector<double> anchor_arm;
};

struct PositionAudit
{
  std::string label;
  double requested = 0.0;
  double modeled_left = 0.0;
  double modeled_right = 0.0;
  Aabb left;
  Aabb right;
  double left_x_overlap = 0.0;
  double left_z_overlap = 0.0;
  double right_x_overlap = 0.0;
  double right_z_overlap = 0.0;
  bool left_side_inside = false;
  bool right_side_inside = false;
};

Contract read_contract(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open " + path);
  Contract contract;
  bool meta = false;
  bool cube = false;
  bool support = false;
  bool translation = false;
  bool gripper = false;
  bool anchor = false;
  std::string line;
  while (std::getline(stream, line))
  {
    if (line.empty())
      continue;
    const auto fields = split(line, '\t');
    if (fields.empty())
      throw std::runtime_error("empty contract row");
    if (fields[0] == "META")
    {
      if (fields.size() != 5 || fields[1] != "1")
        throw std::runtime_error("invalid META row");
      contract.state_count = integer(fields[2]);
      contract.anchor_index = integer(fields[3]);
      contract.reference_link = fields[4];
      meta = true;
    }
    else if (fields[0] == "CUBE" || fields[0] == "SUPPORT")
    {
      if (fields.size() != 7)
        throw std::runtime_error("invalid fixture row");
      const Eigen::Vector3d size(number(fields[1]), number(fields[2]), number(fields[3]));
      const Eigen::Vector3d center(number(fields[4]), number(fields[5]), number(fields[6]));
      if ((size.array() <= 0.0).any())
        throw std::runtime_error("nonpositive fixture size");
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
    else if (fields[0] == "TRANSLATION")
    {
      if (fields.size() != 4)
        throw std::runtime_error("invalid TRANSLATION row");
      contract.translation = Eigen::Vector3d(number(fields[1]), number(fields[2]), number(fields[3]));
      translation = true;
    }
    else if (fields[0] == "GRIPPER")
    {
      if (fields.size() != 5)
        throw std::runtime_error("invalid GRIPPER row");
      contract.lower = number(fields[1]);
      contract.zero = number(fields[2]);
      contract.passive = number(fields[3]);
      contract.upper = number(fields[4]);
      if (!(contract.lower < contract.upper) || contract.zero < contract.lower - TOLERANCE ||
          contract.zero > contract.upper + TOLERANCE || contract.passive < contract.lower - TOLERANCE ||
          contract.passive > contract.upper + TOLERANCE)
        throw std::runtime_error("invalid gripper range");
      gripper = true;
    }
    else if (fields[0] == "ANCHOR")
    {
      if (fields.size() != 8)
        throw std::runtime_error("invalid ANCHOR row");
      contract.anchor_label = fields[1];
      for (std::size_t index = 2; index < fields.size(); ++index)
        contract.anchor_arm.push_back(number(fields[index]));
      anchor = true;
    }
    else
      throw std::runtime_error("unknown contract row: " + fields[0]);
  }
  if (!meta || !cube || !support || !translation || !gripper || !anchor)
    throw std::runtime_error("contract is incomplete");
  if (contract.reference_link != "link7" || contract.state_count != 2802 || contract.anchor_index != 1541 ||
      contract.anchor_label != "sample_001541" || contract.anchor_arm.size() != 6)
    throw std::runtime_error("contract identity mismatch");
  return contract;
}

bool finite_vector(const Eigen::Vector3d& value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

Aabb transformed_link_aabb(
  const moveit::core::RobotState& state,
  const moveit::core::LinkModel* reference,
  const moveit::core::LinkModel* link)
{
  if (link == nullptr || reference == nullptr)
    throw std::runtime_error("required link is missing");
  if (link->getShapes().size() != 1 || link->getShapes().front()->type != shapes::MESH)
    throw std::runtime_error(link->getName() + " must have exactly one collision mesh");
  const Eigen::Vector3d extents = link->getShapeExtentsAtOrigin();
  const Eigen::Vector3d center = link->getCenteredBoundingBoxOffset();
  if (!finite_vector(extents) || !finite_vector(center) || (extents.array() <= 0.0).any())
    throw std::runtime_error(link->getName() + " has invalid collision-mesh AABB");
  const Eigen::Isometry3d reference_link =
    state.getGlobalLinkTransform(reference).inverse() * state.getGlobalLinkTransform(link);
  const Eigen::Matrix3d rotation_error = reference_link.linear() - Eigen::Matrix3d::Identity();
  if (rotation_error.cwiseAbs().maxCoeff() > 1e-12)
    throw std::runtime_error(link->getName() + " is rotated relative to link7");
  Aabb output;
  const Eigen::Vector3d half = extents * 0.5;
  for (int x = -1; x <= 1; x += 2)
    for (int y = -1; y <= 1; y += 2)
      for (int z = -1; z <= 1; z += 2)
      {
        const Eigen::Vector3d local = center + Eigen::Vector3d(x * half.x(), y * half.y(), z * half.z());
        const Eigen::Vector3d point = reference_link * local;
        output.minimum = output.minimum.cwiseMin(point);
        output.maximum = output.maximum.cwiseMax(point);
      }
  if (!finite_vector(output.minimum) || !finite_vector(output.maximum) ||
      (output.maximum.array() <= output.minimum.array()).any())
    throw std::runtime_error(link->getName() + " transformed AABB is invalid");
  return output;
}

double positive_overlap(double first_min, double first_max, double second_min, double second_max)
{
  return std::max(0.0, std::min(first_max, second_max) - std::max(first_min, second_min));
}

bool contains(double minimum, double maximum, double value)
{
  return value >= minimum - TOLERANCE && value <= maximum + TOLERANCE;
}

PositionAudit audit_position(
  const std::string& label,
  double requested,
  moveit::core::RobotState state,
  const moveit::core::LinkModel* reference,
  const moveit::core::LinkModel* left,
  const moveit::core::LinkModel* right,
  const Eigen::Vector3d& cube_min,
  const Eigen::Vector3d& cube_max)
{
  state.setVariablePosition("gripper_left_joint", requested);
  state.update();
  PositionAudit audit;
  audit.label = label;
  audit.requested = requested;
  audit.modeled_left = state.getVariablePosition("gripper_left_joint");
  audit.modeled_right = state.getVariablePosition("gripper_right_joint");
  if (std::abs(audit.modeled_left - requested) > TOLERANCE ||
      std::abs(audit.modeled_right - requested) > TOLERANCE)
    throw std::runtime_error("mimic gripper state mismatch at " + label);
  audit.left = transformed_link_aabb(state, reference, left);
  audit.right = transformed_link_aabb(state, reference, right);
  audit.left_x_overlap = positive_overlap(audit.left.minimum.x(), audit.left.maximum.x(), cube_min.x(), cube_max.x());
  audit.left_z_overlap = positive_overlap(audit.left.minimum.z(), audit.left.maximum.z(), cube_min.z(), cube_max.z());
  audit.right_x_overlap = positive_overlap(audit.right.minimum.x(), audit.right.maximum.x(), cube_min.x(), cube_max.x());
  audit.right_z_overlap = positive_overlap(audit.right.minimum.z(), audit.right.maximum.z(), cube_min.z(), cube_max.z());
  audit.left_side_inside = contains(audit.left.minimum.y(), audit.left.maximum.y(), cube_max.y());
  audit.right_side_inside = contains(audit.right.minimum.y(), audit.right.maximum.y(), cube_min.y());
  return audit;
}

void write_aabb(std::ostream& stream, const Aabb& value)
{
  stream << '\t' << value.minimum.x() << '\t' << value.minimum.y() << '\t' << value.minimum.z()
         << '\t' << value.maximum.x() << '\t' << value.maximum.y() << '\t' << value.maximum.z();
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc != 5)
      throw std::runtime_error("usage: corrected_fixture_relevance URDF SRDF CONTRACT OUTPUT");
    const auto urdf_model = urdf::parseURDF(read_file(argv[1]));
    if (!urdf_model)
      throw std::runtime_error("URDF parsing failed");
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, read_file(argv[2])))
      throw std::runtime_error("SRDF parsing failed");
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    if (robot_model->getRootLinkName() != "base_footprint")
      throw std::runtime_error("unexpected robot root link");
    const auto* arm_group = robot_model->getJointModelGroup("arm");
    const auto* reference = robot_model->getLinkModel("link7");
    const auto* left = robot_model->getLinkModel("gripper_left_link");
    const auto* right = robot_model->getLinkModel("gripper_right_link");
    if (arm_group == nullptr || reference == nullptr || left == nullptr || right == nullptr)
      throw std::runtime_error("required arm or gripper model is missing");
    const std::vector<std::string> expected_arm = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    if (arm_group->getActiveJointModelNames() != expected_arm)
      throw std::runtime_error("active arm joint order mismatch");

    const Contract contract = read_contract(argv[3]);
    moveit::core::RobotState anchor(robot_model);
    anchor.setToDefaultValues();
    anchor.setJointGroupPositions(arm_group, contract.anchor_arm);
    anchor.update();
    if (!anchor.satisfiesBounds(0.0))
      throw std::runtime_error("anchor arm state is out of bounds");

    const Eigen::Vector3d cube_half = contract.cube_size * 0.5;
    const Eigen::Vector3d cube_min = contract.cube_center - cube_half;
    const Eigen::Vector3d cube_max = contract.cube_center + cube_half;
    const std::array<std::pair<std::string, double>, 4> positions = {{
      {"lower", contract.lower}, {"zero", contract.zero},
      {"passive", contract.passive}, {"upper", contract.upper}
    }};
    std::vector<PositionAudit> audits;
    for (const auto& position : positions)
      audits.push_back(audit_position(
        position.first, position.second, anchor, reference, left, right, cube_min, cube_max));

    const PositionAudit& zero = audits[1];
    const PositionAudit& lower = audits[0];
    const PositionAudit& upper = audits[3];
    double linearity_residual = 0.0;
    for (int axis : {0, 2})
    {
      linearity_residual = std::max(linearity_residual, std::abs(lower.left.minimum[axis] - zero.left.minimum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(lower.left.maximum[axis] - zero.left.maximum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(upper.left.minimum[axis] - zero.left.minimum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(upper.left.maximum[axis] - zero.left.maximum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(lower.right.minimum[axis] - zero.right.minimum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(lower.right.maximum[axis] - zero.right.maximum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(upper.right.minimum[axis] - zero.right.minimum[axis]));
      linearity_residual = std::max(linearity_residual, std::abs(upper.right.maximum[axis] - zero.right.maximum[axis]));
    }
    for (const auto& audit : audits)
    {
      linearity_residual = std::max(
        linearity_residual, std::abs((audit.left.minimum.y() - zero.left.minimum.y()) - audit.requested));
      linearity_residual = std::max(
        linearity_residual, std::abs((audit.left.maximum.y() - zero.left.maximum.y()) - audit.requested));
      linearity_residual = std::max(
        linearity_residual, std::abs((audit.right.minimum.y() - zero.right.minimum.y()) + audit.requested));
      linearity_residual = std::max(
        linearity_residual, std::abs((audit.right.maximum.y() - zero.right.maximum.y()) + audit.requested));
    }
    if (linearity_residual > TOLERANCE)
      throw std::runtime_error("gripper AABB motion is not the required linear mimic translation");

    const double left_low = cube_max.y() - zero.left.maximum.y();
    const double left_high = cube_max.y() - zero.left.minimum.y();
    const double right_low = zero.right.minimum.y() - cube_min.y();
    const double right_high = zero.right.maximum.y() - cube_min.y();
    const double common_low = std::max({contract.lower, left_low, right_low});
    const double common_high = std::min({contract.upper, left_high, right_high});
    const bool common_exists = common_low <= common_high + TOLERANCE;
    const double representative = common_exists ? 0.5 * (common_low + common_high) : contract.zero;
    const PositionAudit common = audit_position(
      "common_interval_midpoint", representative, anchor, reference, left, right, cube_min, cube_max);

    const bool positive_xz_overlap =
      zero.left_x_overlap > 0.0 && zero.left_z_overlap > 0.0 &&
      zero.right_x_overlap > 0.0 && zero.right_z_overlap > 0.0;
    const bool representative_reaches = common_exists && common.left_side_inside && common.right_side_inside;
    const bool relevant = positive_xz_overlap && representative_reaches;
    const std::string decision = relevant ?
      "CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT" :
      "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE";

    std::ofstream output(argv[4]);
    if (!output)
      throw std::runtime_error("cannot open output TSV");
    output << std::setprecision(17);
    output << "META\t1\t" << robot_model->getName() << '\t' << robot_model->getRootLinkName()
           << '\t' << contract.anchor_label << '\t' << contract.reference_link << '\n';
    output << "CUBE\t" << contract.cube_size.x() << '\t' << contract.cube_size.y() << '\t'
           << contract.cube_size.z() << '\t' << contract.cube_center.x() << '\t'
           << contract.cube_center.y() << '\t' << contract.cube_center.z() << '\n';
    output << "MESH\tgripper_left_link\t" << left->getShapes().size() << '\t'
           << left->getCenteredBoundingBoxOffset().x() << '\t' << left->getCenteredBoundingBoxOffset().y() << '\t'
           << left->getCenteredBoundingBoxOffset().z() << '\t' << left->getShapeExtentsAtOrigin().x() << '\t'
           << left->getShapeExtentsAtOrigin().y() << '\t' << left->getShapeExtentsAtOrigin().z() << '\n';
    output << "MESH\tgripper_right_link\t" << right->getShapes().size() << '\t'
           << right->getCenteredBoundingBoxOffset().x() << '\t' << right->getCenteredBoundingBoxOffset().y() << '\t'
           << right->getCenteredBoundingBoxOffset().z() << '\t' << right->getShapeExtentsAtOrigin().x() << '\t'
           << right->getShapeExtentsAtOrigin().y() << '\t' << right->getShapeExtentsAtOrigin().z() << '\n';
    for (const auto& audit : audits)
    {
      output << "POSITION\t" << audit.label << '\t' << audit.requested << '\t'
             << audit.modeled_left << '\t' << audit.modeled_right;
      write_aabb(output, audit.left);
      write_aabb(output, audit.right);
      output << '\t' << audit.left_x_overlap << '\t' << audit.left_z_overlap
             << '\t' << audit.right_x_overlap << '\t' << audit.right_z_overlap
             << '\t' << (audit.left_side_inside ? "true" : "false")
             << '\t' << (audit.right_side_inside ? "true" : "false") << '\n';
    }
    output << "INTERVAL\t" << left_low << '\t' << left_high << '\t'
           << right_low << '\t' << right_high << '\t' << common_low << '\t' << common_high
           << '\t' << (common_exists ? "true" : "false") << '\t' << representative
           << '\t' << (common.left_side_inside ? "true" : "false")
           << '\t' << (common.right_side_inside ? "true" : "false") << '\n';
    output << "DECISION\t" << decision << '\t' << (positive_xz_overlap ? "true" : "false")
           << '\t' << (representative_reaches ? "true" : "false")
           << '\t' << linearity_residual << '\n';
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
