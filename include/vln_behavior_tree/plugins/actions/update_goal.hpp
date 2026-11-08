#ifndef VLN_BEHAVIOR_TREE__PLUGINS__ACTIONS__UPDATE_GOAL_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__ACTIONS__UPDATE_GOAL_HPP_

#include <map>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vln_behavior_tree
{
/** @brief 为目标数值表达式提供自定义变量。 */
using GoalVariables = std::map<std::string, double>;

/** @brief 使用同一份原始目标求值，统一写回位置及偏航角。 */
class UpdateGoal : public BT::SyncActionNode
{
public:
  UpdateGoal(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  rclcpp::Node::WeakPtr node_;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__ACTIONS__UPDATE_GOAL_HPP_
