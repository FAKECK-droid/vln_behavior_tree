#ifndef VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_SURROUNDINGS_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_SURROUNDINGS_HPP_

#include "vln_behavior_tree/call_tool_node_base.hpp"

namespace vln_behavior_tree
{
/** @brief 通过 ToolCall 获取环境图，保留原生 function_call_output。 */
class GetSurroundings : public CallToolNodeBase
{
public:
  using CallToolNodeBase::CallToolNodeBase;
  static BT::PortsList providedPorts();

protected:
  std::string toolName() override;
  Json buildArguments() override;
  void on_tick() override;
  BT::NodeStatus onOutput(Json output) override;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_SURROUNDINGS_HPP_
