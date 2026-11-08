#ifndef VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
#define VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "vln_behavior_tree/image_content.hpp"

namespace tf2_ros
{
class Buffer;
class TransformListener;
}

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

  ~EventNodeBase() override;

  static BT::PortsList providedPorts();

protected:
  /** @brief 构建统一业务字段、data 载荷、图片元数据和 Base64 图片内容。 */
  virtual EventPayload buildPayload();
  /** @brief 使用派生类准备好的图片集合，统一构建事件及图片编码。 */
  EventPayload buildPayload(const ImageContent & images);

private:
  BT::NodeStatus tick() override;
  /** @brief 从 map 到 base_link 的最新 TF 获取位姿，失败时保留事件其余信息。 */
  std::optional<Pose2D> lookupRobotPose();

  rclcpp::Node::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__EVENT_NODE_BASE_HPP_
