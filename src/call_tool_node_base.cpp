#include "vln_behavior_tree/call_tool_node_base.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <random>
#include <sstream>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_behavior_tree/bt_conversions.hpp>

namespace vln_behavior_tree
{
namespace
{
/** @brief 用随机进程前缀和原子计数生成机器人调用 ID，区分其他调用方和重复执行。 */
std::string nextCallId()
{
  static const std::string prefix = [] {
      std::random_device random;
      std::ostringstream text;
      text << "robot_bt_" << std::hex << std::setfill('0');
      for (int i = 0; i < 4; ++i) {
        text << std::setw(8) << random();
      }
      return text.str() + "_";
    }();
  static std::atomic<std::uint64_t> sequence{0};
  return prefix + std::to_string(++sequence);
}
}  // namespace

CallToolNodeBase::CallToolNodeBase(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  callback_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

BT::PortsList CallToolNodeBase::providedPorts()
{
  BT::PortsList ports{
    BT::InputPort<std::string>("tool", "Agent 中启用的工具函数名"),
    BT::InputPort<std::string>("arguments", "{}", "工具参数 JSON 对象"),
    BT::OutputPort<ToolResult>("result", "工具业务 payload 及原生图片"),
    BT::InputPort<std::string>("service_name", "/vln/tool_call", "Agent 工具调用服务"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout", "工具服务与结果的总等待时间，单位毫秒")};
  ports.at("server_timeout").setDefaultValue("5000");
  return ports;
}

std::string CallToolNodeBase::toolName()
{
  const auto tool = getInput<std::string>("tool");
  if (!tool || tool.value().empty()) {throw BT::RuntimeError("请提供工具函数名 tool");}
  return tool.value();
}

CallToolNodeBase::Json CallToolNodeBase::buildArguments()
{
  const auto arguments = getInput<std::string>("arguments");
  if (!arguments) {throw BT::RuntimeError("工具参数输入无效: ", arguments.error());}
  return Json::parse(arguments.value());
}

void CallToolNodeBase::on_tick()
{
  if (config().output_ports.count("result")) {setOutput("result", ToolResult{});}
  const auto tool = toolName();
  auto arguments = buildArguments();
  if (!arguments.is_object()) {throw BT::RuntimeError("工具 arguments 需要为 JSON 对象");}
  call_id_ = nextCallId();
  request_->request_json = Json{
    {"status", "completed"}, {"output", Json::array(
        {
          {{"type", "function_call"}, {"call_id", call_id_},
            {"name", tool}, {"arguments", arguments.dump()}}})}}.dump();
}

BT::NodeStatus CallToolNodeBase::on_completion(Service::Response::SharedPtr response)
{
  if (!response->error.empty()) {throw BT::RuntimeError("ToolCall 服务错误: ", response->error);}
  auto outputs = Json::parse(response->outputs_json);
  if (!outputs.is_array() || outputs.size() != 1) {
    throw BT::RuntimeError("ToolCall 服务需要返回本次调用的一项工具结果");
  }
  auto & output = outputs.at(0);
  if (output.at("type") != "function_call_output" || output.at("call_id") != call_id_) {
    throw BT::RuntimeError("ToolCall 返回的工具结果与当前调用不对应");
  }
  return onOutput(std::move(output));
}

BT::NodeStatus CallToolNodeBase::onOutput(Json output)
{
  auto & content = output.at("output");
  if (!content.is_array() || content.empty() || content.at(0).at("type") != "input_text") {
    throw BT::RuntimeError("ToolCall 结果需要包含首个 input_text 业务载荷");
  }
  ToolResult result{Json::parse(content.at(0).at("text").get<std::string>()), {}};
  for (std::size_t i = 1; i < content.size(); ++i) {
    result.images.push_back(std::move(content.at(i)));
  }
  vln_agent::utils::validateImageMetadata(result.payload, result.images.size());
  if (config().output_ports.count("result")) {setOutput("result", result);}
  return onResult(result);
}

BT::NodeStatus CallToolNodeBase::onResult(const ToolResult & result)
{
  return result.payload.at("ok").get<bool>() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CallToolNodeBase::tick()
{
  try {
    const auto now = std::chrono::steady_clock::now();
    if (deadline_ == std::chrono::steady_clock::time_point{}) {
      const auto service_name = getInput<std::string>("service_name").value();
      if (!client_ || service_name != service_name_) {
        client_ = node_->create_client<Service>(
          service_name, rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);
        service_name_ = service_name;
      }
      on_tick();
      deadline_ = now +
        config().blackboard->get<std::chrono::milliseconds>("wait_for_service_timeout");
    }
    if (!pending_ && client_->service_is_ready()) {
      pending_.emplace(client_->async_send_request(request_));
      deadline_ = now + getInput<std::chrono::milliseconds>("server_timeout").value();
    }
    if (pending_) {
      executor_.spin_some();
      if (pending_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        const auto status = on_completion(pending_->future.get());
        resetRequest();
        return status;
      }
    }
    if (now < deadline_) {return BT::NodeStatus::RUNNING;}
    RCLCPP_WARN(node_->get_logger(), "工具节点 '%s' 等待结果超时", name().c_str());
    resetRequest();
    return BT::NodeStatus::FAILURE;
  } catch (const std::exception & error) {
    resetRequest();
    RCLCPP_ERROR(node_->get_logger(), "工具节点 '%s' 调用失败: %s", name().c_str(), error.what());
    return BT::NodeStatus::FAILURE;
  }
}

void CallToolNodeBase::resetRequest()
{
  if (pending_) {client_->remove_pending_request(*pending_);}
  pending_.reset();
  deadline_ = {};
  request_->request_json.clear();
}

void CallToolNodeBase::halt()
{
  resetRequest();
  setStatus(BT::NodeStatus::IDLE);
}
}  // namespace vln_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::CallToolNodeBase>("CallTool");
}
