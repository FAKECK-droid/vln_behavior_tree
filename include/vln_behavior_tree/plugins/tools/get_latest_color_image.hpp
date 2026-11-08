#ifndef VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_

#include <string>

#include "vln_behavior_tree/call_tool_node_base.hpp"
#include "vln_behavior_tree/image_content_utils.hpp"

namespace vln_behavior_tree
{
/** @brief 调用 Agent 获取最新彩色图，更新黑板中的图片集合。 */
class GetLatestColorImage : public CallToolNodeBase
{
public:
  using CallToolNodeBase::CallToolNodeBase;

  /** @brief operation 控制集合操作，image_content 同时作为输入和输出。 */
  static BT::PortsList providedPorts();
  /** @brief clear 直接清空集合；其他操作复用异步工具服务调用。 */
  BT::NodeStatus tick() override;

protected:
  std::string toolName() override;
  Json buildArguments() override;
  BT::NodeStatus onResult(const ToolResult & result) override;

private:
  ImageOperation operation_;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_
