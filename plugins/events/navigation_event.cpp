#include "vln_behavior_tree/plugins/events/navigation_event.hpp"

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <vln_agent/utils/json.hpp>
#include <vln_agent/utils/time.hpp>

namespace vln_behavior_tree
{
namespace
{
/** @brief 默认黑板项尚未提供时省略对应数据；已提供的输入按声明类型读取。 */
template<typename T>
std::optional<T> readOptional(const BT::TreeNode & node, const std::string & port)
{
  const auto & config = node.config();
  const auto mapping = config.input_ports.find(port);
  if (mapping == config.input_ports.end()) {return std::nullopt;}
  const auto key = BT::TreeNode::getRemappedKey(port, mapping->second);
  if (key) {
    std::unique_lock<std::mutex> lock(config.blackboard->entryMutex());
    const auto value = config.blackboard->getAny(std::string(key.value()));
    if (!value || value->empty()) {return std::nullopt;}
  }
  const auto value = node.getInput<T>(port);
  if (!value) {throw BT::RuntimeError("导航事件输入 '", port, "' 无效: ", value.error());}
  return value.value();
}

std::string sampleTime(const builtin_interfaces::msg::Time & stamp, bool real_time)
{
  if (!real_time || (stamp.sec == 0 && stamp.nanosec == 0)) {return {};}
  return vln_agent::utils::formatUtcTime(
    std::chrono::system_clock::time_point(
      std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
}

Json poseJson(const geometry_msgs::msg::PoseStamped & pose, bool real_time)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  auto result = vln_agent::utils::posePayload(
    pose.header.frame_id,
    p.x, p.y, tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w)),
    sampleTime(pose.header.stamp, real_time));
  result["z"] = p.z;
  return result;
}
}  // namespace

BT::PortsList NavigationEvent::providedPorts()
{
  auto ports = EventNodeBase::providedPorts();
  ports.insert(BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "当前单目标导航位姿"));
  ports.insert(BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "当前多目标序列"));
  ports.insert(BT::InputPort<int>("number_recoveries", "导航恢复次数"));
  for (const auto & key : {"goal", "goals", "number_recoveries"}) {
    ports.at(key).setDefaultValue("{" + std::string(key) + "}");
  }
  return ports;
}

EventPayload NavigationEvent::buildPayload()
{
  auto event = EventNodeBase::buildPayload();
  auto & data = event.business["data"];
  bool simulated = false;
  const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter_or("use_sim_time", simulated, false);
  if (const auto goal = readOptional<geometry_msgs::msg::PoseStamped>(*this, "goal")) {
    data["goal"] = poseJson(*goal, !simulated);
  }
  if (const auto goals =
    readOptional<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "goals"))
  {
    data["goals"] = Json::array();
    for (const auto & goal : *goals) {
      data["goals"].push_back(poseJson(goal, !simulated));
    }
  }
  if (const auto count = readOptional<int>(*this, "number_recoveries")) {
    data["number_recoveries"] = *count;
  }
  return event;
}
}  // namespace vln_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::NavigationEvent>("NavigationEvent");
}
