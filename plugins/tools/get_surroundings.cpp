#include "vln_behavior_tree/plugins/tools/get_surroundings.hpp"

#include <utility>

namespace vln_behavior_tree
{
BT::PortsList GetSurroundings::providedPorts()
{
  auto ports = CallToolNodeBase::providedPorts();
  ports.erase("tool");
  ports.erase("arguments");
  ports.erase("result");
  ports.insert(BT::InputPort<std::string>("costmap", "local", "代价地图来源，local 或 global"));
  ports.insert(BT::InputPort<double>("range_m", 6.0, "观察区域边长，单位米"));
  ports.insert(BT::OutputPort<Json>("output", "工具返回的原生 function_call_output"));
  return ports;
}

std::string GetSurroundings::toolName() {return "get_surroundings";}

GetSurroundings::Json GetSurroundings::buildArguments()
{
  return {{"costmap", getInput<std::string>("costmap").value()},
    {"range_m", getInput<double>("range_m").value()}};
}

void GetSurroundings::on_tick()
{
  setOutput("output", Json());
  CallToolNodeBase::on_tick();
}

BT::NodeStatus GetSurroundings::onOutput(Json output)
{
  const auto payload = Json::parse(
    output.at("output").at(0).at("text").get_ref<const std::string &>());
  const bool ok = payload.at("ok").get<bool>();
  setOutput("output", std::move(output));
  if (!ok) {
    RCLCPP_WARN(node_->get_logger(), "获取代价地图失败: %s", payload.dump().c_str());
  }
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace vln_behavior_tree
