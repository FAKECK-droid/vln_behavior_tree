#ifndef VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
#define VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace vln_behavior_tree
{
using Json = nlohmann::json;

/** @brief 从输入构建机器事件并同步发布；可直接注册或派生扩展业务端口。 */
class EventNodeBase : public BT::SyncActionNode
{
public:
  EventNodeBase(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

protected:
  /** @brief 构建完整业务 payload；派生类可调用本实现后添加业务字段。 */
  virtual Json buildPayload();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_;
};
}  // namespace vln_behavior_tree

namespace BT
{
/** @brief 让图片端口同时支持黑板 JSON 值和 JSON 数组文本。 */
template<>
inline vln_behavior_tree::Json convertFromString<vln_behavior_tree::Json>(StringView text)
{
  return vln_behavior_tree::Json::parse(text.begin(), text.end());
}
}  // namespace BT

#endif  // VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
