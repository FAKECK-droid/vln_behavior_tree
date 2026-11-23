#ifndef VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_
#define VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <vln_agent/srv/tool_call.hpp>
#include <vln_agent/tools/tool_result.hpp>

namespace vln_behavior_tree
{
/** @brief 通过 Agent 的 ToolCall 服务执行工具，结果通过黑板传递。 */
class CallToolNodeBase : public BT::ActionNodeBase
{
public:
  using Service = vln_agent::srv::ToolCall;
  using Json = vln_agent::Json;
  using ToolResult = vln_agent::ToolResult;

  /**
   * @brief 创建回调组；在 tick 中等待 ToolCall 服务就绪。
   * @param name 行为树节点实例名。
   * @param config Nav2 提供的 ROS 节点和端口。
   */
  CallToolNodeBase(const std::string & name, const BT::NodeConfiguration & config);

  /** @brief 提供 tool、arguments、result 及 Nav2 服务端口；结果等待默认 5000 毫秒。 */
  static BT::PortsList providedPorts();

  /** @brief 异步轮询工具输出；构建、服务和响应解析异常返回 FAILURE。 */
  BT::NodeStatus tick() override;
  /** @brief 停止本地等待并清理挂起请求；已提交的工具继续由 Agent 执行。 */
  void halt() override;

protected:
  /** @brief 提供工具名；默认读取 tool，专用节点可返回固定名称。 */
  virtual std::string toolName();
  /** @brief 构建本次工具参数；默认解析 arguments，派生类可读取自己的类型化端口。 */
  virtual Json buildArguments();
  /**
   * @brief 结果已写入输出端口后处理本次工具结果。
   * @param result 保留公共业务 payload 与原生图片的工具结果。
   * @return 默认按 payload.ok 返回 SUCCESS 或 FAILURE。
   */
  virtual BT::NodeStatus onResult(const ToolResult & result);
  /** @brief 处理已关联到本次调用的原生 function_call_output。 */
  virtual BT::NodeStatus onOutput(Json output);

  /** @brief 清空上次输出，封装本次 function_call 并填写 request_json。 */
  virtual void on_tick();

  rclcpp::Node::SharedPtr node_;

private:
  /** @brief 解析服务响应、关联本次调用并交给结果处理函数。 */
  BT::NodeStatus on_completion(Service::Response::SharedPtr response);
  /** @brief 释放当前请求，供结束、异常及 halt 共用。 */
  void resetRequest();

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  Service::Request::SharedPtr request_ = std::make_shared<Service::Request>();
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  std::chrono::steady_clock::time_point deadline_;
  std::string service_name_;
  std::string call_id_;  ///< 当前调用的独立关联 ID，每次发起请求重新生成。
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_
