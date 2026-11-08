#include "vln_behavior_tree/plugins/actions/update_goal.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tinyexpr.h>

namespace vln_behavior_tree
{
namespace
{
double minimum(double a, double b) {return std::min(a, b);}
double maximum(double a, double b) {return std::max(a, b);}

template<typename T>
T readInput(const BT::TreeNode & node, const std::string & port)
{
  const auto input = node.getInput<T>(port);
  if (!input) {throw BT::RuntimeError("UpdateGoal 输入 '", port, "' 无效: ", input.error());}
  return input.value();
}

/** @brief 留空的字段保持原值，每个表达式均绑定到未修改的目标与变量快照。 */
double evaluate(
  const BT::TreeNode & node, const std::string & port, double original,
  const std::vector<te_variable> & variables)
{
  if (!node.config().input_ports.count(port)) {return original;}
  const auto text = readInput<std::string>(node, port);
  if (text.find_first_not_of(" \t\r\n") == std::string::npos) {return original;}
  int position = 0;
  const std::unique_ptr<te_expr, decltype(& te_free)> expression(
    te_compile(text.c_str(), variables.data(), static_cast<int>(variables.size()), &position),
    te_free);
  if (!expression) {
    throw BT::RuntimeError("字段 '", port, "' 表达式解析失败，位置 ", std::to_string(position));
  }
  const double value = te_eval(expression.get());
  if (!std::isfinite(value)) {throw BT::RuntimeError("字段 '", port, "' 的计算结果需要为有限数值");}
  return value;
}
}  // namespace

UpdateGoal::UpdateGoal(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard) {throw BT::RuntimeError("UpdateGoal 需要 Nav2 黑板");}
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (node_.expired()) {throw BT::RuntimeError("UpdateGoal 的 ROS node 为空");}
}

BT::PortsList UpdateGoal::providedPorts()
{
  return {
    BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("goal", "读取并更新的导航目标"),
    BT::InputPort<std::string>("x", "", "x 坐标表达式，空值保留原坐标"),
    BT::InputPort<std::string>("y", "", "y 坐标表达式，空值保留原坐标"),
    BT::InputPort<std::string>("z", "", "z 坐标表达式，空值保留原坐标"),
    BT::InputPort<std::string>("yaw", "", "偏航角表达式，单位弧度，空值保留原姿态"),
    BT::InputPort<GoalVariables>("variables", "表达式的自定义数值变量")};
}

BT::NodeStatus UpdateGoal::tick()
{
  const auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("vln_behavior_tree"), "UpdateGoal 的 ROS node 已释放");
    return BT::NodeStatus::FAILURE;
  }
  try {
    const auto original = readInput<geometry_msgs::msg::PoseStamped>(*this, "goal");
    auto updated = original;
    const auto & rotation = original.pose.orientation;
    const double norm = std::hypot(
      std::hypot(rotation.x, rotation.y),
      std::hypot(rotation.z, rotation.w));
    if (!std::isfinite(norm) || norm == 0.0) {throw BT::RuntimeError("输入目标的四元数无效");}
    const tf2::Quaternion quaternion(rotation.x / norm, rotation.y / norm,
      rotation.z / norm, rotation.w / norm);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    auto values = config().input_ports.count("variables") ?
      readInput<GoalVariables>(*this, "variables") : GoalVariables{};
    values["x"] = original.pose.position.x;
    values["y"] = original.pose.position.y;
    values["z"] = original.pose.position.z;
    values["yaw"] = yaw;
    values["pi"] = std::acos(-1.0);
    std::vector<te_variable> variables;
    variables.reserve(values.size() + 2);
    for (const auto & value : values) {
      variables.push_back({value.first.c_str(), &value.second, TE_VARIABLE, nullptr});
    }
    variables.push_back(
      {"min", reinterpret_cast<const void *>(&minimum),
        TE_FUNCTION2 | TE_FLAG_PURE, nullptr});
    variables.push_back(
      {"max", reinterpret_cast<const void *>(&maximum),
        TE_FUNCTION2 | TE_FLAG_PURE, nullptr});

    auto & position = updated.pose.position;
    position.x = evaluate(*this, "x", values.at("x"), variables);
    position.y = evaluate(*this, "y", values.at("y"), variables);
    position.z = evaluate(*this, "z", values.at("z"), variables);
    const double new_yaw = evaluate(*this, "yaw", yaw, variables);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      throw BT::RuntimeError("目标位置需要为有限数值");
    }
    const bool rotated = new_yaw != yaw;
    const bool moved = position != original.pose.position;
    if (!moved && !rotated) {return BT::NodeStatus::SUCCESS;}
    if (rotated) {
      tf2::Quaternion updated_rotation;
      updated_rotation.setRPY(roll, pitch, new_yaw);
      updated_rotation.normalize();
      updated.pose.orientation.x = updated_rotation.x();
      updated.pose.orientation.y = updated_rotation.y();
      updated.pose.orientation.z = updated_rotation.z();
      updated.pose.orientation.w = updated_rotation.w();
    }
    updated.header.stamp = node->now();
    const auto result = setOutput("goal", updated);
    if (!result) {throw BT::RuntimeError("目标输出失败: ", result.error());}
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "目标更新节点 '%s' 失败: %s", name().c_str(), error.what());
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vln_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::UpdateGoal>("UpdateGoal");
}
