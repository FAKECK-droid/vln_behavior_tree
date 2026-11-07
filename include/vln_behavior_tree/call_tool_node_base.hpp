#ifndef VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_
#define VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_

#include <string>

#include <nav2_behavior_tree/bt_service_node.hpp>
#include <vln_agent/srv/tool_call.hpp>
#include <vln_agent/tools/tool_result.hpp>

namespace vln_behavior_tree
{
/** @brief 通过 Agent 的 ToolCall 服务执行工具，结果通过黑板传递。 */
class CallToolNodeBase : public nav2_behavior_tree::BtServiceNode<vln_agent::srv::ToolCall>
{
public:
  using Service = vln_agent::srv::ToolCall;
  using Base = nav2_behavior_tree::BtServiceNode<Service>;
  using Json = vln_agent::Json;
  using ToolResult = vln_agent::ToolResult;

  /**
   * @brief 创建 Nav2 服务客户端，默认连接 /vln/tool_call。
   * @param name 行为树节点实例名。
   * @param config Nav2 提供的黑板、端口和服务等待配置。
   */
  CallToolNodeBase(const std::string & name, const BT::NodeConfiguration & config);

  /** @brief 提供 tool、arguments、result 及 Nav2 服务端口；结果等待默认 5000 毫秒。 */
  static BT::PortsList providedPorts();

  /** @brief 沿用 Nav2 异步等待；构建、服务和响应解析异常返回 FAILURE。 */
  BT::NodeStatus tick() override;
  /** @brief 停止本地等待并清理挂起请求；已提交的工具继续由 Agent 执行。 */
  void halt() override;

protected:
  /** @brief 构建本次工具参数；默认解析 arguments，派生类可读取自己的类型化端口。 */
  virtual Json buildArguments();
  /**
   * @brief 结果已写入输出端口后处理本次工具结果。
   * @param result 保留公共业务 payload 与原生图片的工具结果。
   * @return 默认按 payload.ok 返回 SUCCESS 或 FAILURE。
   */
  virtual BT::NodeStatus onResult(const ToolResult & result);

  /** @brief 清空上次输出，封装本次 function_call 并填写 request_json。 */
  void on_tick() override;
  /** @brief 解析服务响应、关联本次调用并写入 ToolResult 输出。 */
  BT::NodeStatus on_completion(Service::Response::SharedPtr response) override;

private:
  /** @brief 释放本客户端挂起请求和响应正文，供结束、异常及 halt 共用。 */
  void resetRequest();

  std::string call_id_;  ///< 当前调用的独立关联 ID，每次发起请求重新生成。
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__CALL_TOOL_NODE_BASE_HPP_
