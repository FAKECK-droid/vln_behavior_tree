#include "vln_behavior_tree/event_node_base.hpp"

#include <exception>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <vln_agent/utils/base64.hpp>
#include <vln_agent/utils/json.hpp>

namespace vln_behavior_tree
{
namespace
{
/** @brief 使用共用格式序列化事件或图片采样时的小车位姿。 */
Json poseJson(const Pose2D & pose)
{
  return vln_agent::utils::posePayload(
    pose.frame_id, pose.x, pose.y, pose.yaw, pose.time_stamp);
}

template<typename T>
T readInput(const BT::TreeNode & node, const std::string & port)
{
  const auto input = node.getInput<T>(port);
  if (!input) {
    throw BT::RuntimeError("Event 输入 '", port, "' 无效: ", input.error());
  }
  return input.value();
}
}  // namespace

EventNodeBase::EventNodeBase(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError("Event 节点需要 Nav2 黑板中的 node");
  }
  const auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node) {
    throw BT::RuntimeError("Event 节点的 ROS node 为空");
  }
  node_ = node;
}

BT::PortsList EventNodeBase::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/vln/robot_event", "机器事件接收话题"),
    BT::InputPort<std::string>("event", "status", "事件类型，与 data 同级"),
    BT::InputPort<std::string>("message", "", "事件说明，写入 data.message"),
    BT::InputPort<std::string>("time_stamp", "可选的事件发生时间，UTC 三位毫秒字符串"),
    BT::InputPort<Pose2D>("pose", "可选的小车位姿"),
    BT::InputPort<ImageContent>("image_content", "图片字节、采样信息及自定义元数据")};
}

EventPayload EventNodeBase::buildPayload()
{
  EventPayload result;
  auto & payload = result.business;
  payload = vln_agent::utils::businessPayload("robot");
  payload["origin"] = name();
  payload["event"] = readInput<std::string>(*this, "event");
  // BT 3.8 会省略空字符串默认端口的映射。
  const auto message = config().input_ports.count("message") ?
    readInput<std::string>(*this, "message") : std::string();
  if (!message.empty()) {payload["data"]["message"] = message;}
  if (config().input_ports.count("time_stamp")) {
    payload["time_stamp"] = readInput<std::string>(*this, "time_stamp");
  }
  if (config().input_ports.count("pose")) {
    payload["pose"] = poseJson(readInput<Pose2D>(*this, "pose"));
  }

  if (config().input_ports.count("image_content")) {
    const auto input = readInput<ImageContent>(*this, "image_content");
    if (!input.images.empty()) {payload["images"] = Json::array();}
    for (const auto & image : input.images) {
      if (image.bytes.empty()) {
        throw BT::RuntimeError("图片字节数组为空");
      }
      Json metadata = Json::object();
      for (const auto & field : image.metadata) {
        std::visit([&](const auto & value) {metadata[field.first] = value;}, field.second);
      }
      Json description{{"metadata", std::move(metadata)}};
      if (!image.time_stamp.empty()) {description["time_stamp"] = image.time_stamp;}
      if (image.pose) {description["pose"] = poseJson(*image.pose);}
      payload["images"].push_back(std::move(description));
      result.image_content.push_back(
        {{"type", "input_image"},
          {"image_url", "data:" + image.mime_type + ";base64," +
            vln_agent::utils::encodeBase64(image.bytes)}, {"detail", "auto"}});
    }
  }
  return result;
}

BT::NodeStatus EventNodeBase::tick()
{
  const auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(
      rclcpp::get_logger("vln_behavior_tree"),
      "Event 节点 '%s' 的 ROS node 已释放", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  try {
    auto event = buildPayload();
    vln_agent::utils::validateImageMetadata(event.business, event.image_content.size());
    const Json text{{"type", "input_text"}, {"text", event.business.dump()}};
    event.image_content.insert(event.image_content.begin(), text);
    const Json message{{"role", "user"}, {"content", std::move(event.image_content)}};
    std_msgs::msg::String output;
    output.data = message.dump();

    const auto topic = readInput<std::string>(*this, "topic");
    if (!publisher_ || topic_ != topic) {
      auto publisher = node->create_publisher<std_msgs::msg::String>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
      publisher_ = std::move(publisher);
      topic_ = topic;
    }
    publisher_->publish(output);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node->get_logger(), "Event 节点 '%s' 上报失败: %s",
      name().c_str(), error.what());
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vln_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::EventNodeBase>("SubmitEvent");
}
