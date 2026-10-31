#include "vln_behavior_tree/event_node_base.hpp"

#include <chrono>
#include <ctime>
#include <exception>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>

namespace vln_behavior_tree
{
namespace
{
std::string utcNow()
{
  using namespace std::chrono;
  const auto now = system_clock::now().time_since_epoch();
  const auto seconds_part = floor<seconds>(now);
  const auto milliseconds_part = duration_cast<milliseconds>(now - seconds_part).count();
  const auto raw = static_cast<std::time_t>(seconds_part.count());
  std::tm utc{};
  if (!gmtime_r(&raw, &utc)) {
    throw std::runtime_error("UTC 时间无法格式化");
  }
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S.")
       << std::setfill('0') << std::setw(3) << milliseconds_part << 'Z';
  return text.str();
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
  auto images = BT::InputPort<Json>("image_content", "原生图片内容数组，可附带 camera/time_stamp");
  images.second.setDefaultValue("[]");
  return {
    BT::InputPort<std::string>("topic", "/vln/robot_event", "机器事件接收话题"),
    BT::InputPort<std::string>("event", "status", "事件类型"),
    BT::InputPort<std::string>("text", "", "事件说明文字"),
    std::move(images)};
}

Json EventNodeBase::buildPayload()
{
  Json payload{{"source", "robot"}, {"time_stamp", utcNow()}, {"origin", name()},
    {"event", readInput<std::string>(*this, "event")}};
  // BT 3.8 会省略空字符串默认端口的映射。
  const auto text = config().input_ports.count("text") ?
    readInput<std::string>(*this, "text") : std::string();
  if (!text.empty()) {payload["text"] = text;}

  const auto images = readInput<Json>(*this, "image_content");
  if (!images.is_array()) {
    throw BT::RuntimeError("image_content 应为 JSON 数组");
  }
  if (!images.empty()) {
    payload["images"] = Json::array();
    for (const auto & image : images) {
      Json metadata = Json::object();
      for (const auto * field : {"camera", "time_stamp"}) {
        if (image.contains(field)) {metadata[field] = image.at(field);}
      }
      payload["images"].push_back(std::move(metadata));
    }
  }
  return payload;
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
    const auto payload = buildPayload();
    Json content = Json::array({{{"type", "input_text"}, {"text", payload.dump()}}});
    const auto images = readInput<Json>(*this, "image_content");
    if (!images.is_array()) {
      throw BT::RuntimeError("image_content 应为 JSON 数组");
    }
    for (auto image : images) {
      // 采样元数据已进入业务 payload，图片项只保留原生内容字段。
      if (!image.is_object() || image.value("type", std::string()) != "input_image" ||
        !image.contains("image_url") || !image.at("image_url").is_string())
      {
        throw BT::RuntimeError("image_content 的每项应包含 input_image 类型和 image_url");
      }
      image.erase("camera");
      image.erase("time_stamp");
      content.push_back(std::move(image));
    }
    const Json message{{"role", "user"}, {"content", std::move(content)}};
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
