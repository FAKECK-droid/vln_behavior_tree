#ifndef VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
#define VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "vln_behavior_tree/image_content.hpp"

namespace vln_behavior_tree
{
using Json = nlohmann::json;

/** @brief 一次构建完成的业务数据和原生图片内容。 */
struct EventPayload
{
  Json business = Json::object();
  Json image_content = Json::array();
};

/** @brief 从输入构建机器事件并同步发布；可直接注册或派生扩展业务端口。 */
class EventNodeBase : public BT::SyncActionNode
{
public:
  EventNodeBase(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

protected:
  /** @brief 读取输入并构建业务数据、图片元数据和 Base64 图片内容。 */
  virtual EventPayload buildPayload();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
