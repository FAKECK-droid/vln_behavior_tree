#ifndef VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_

#include <cstdint>
#include <string>

#include "vln_behavior_tree/call_tool_node_base.hpp"
#include "vln_behavior_tree/image_content.hpp"

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
  /** @brief 读取已有图片；首次使用的黑板变量从空集合开始。 */
  ImageContent readImages() const;
  /** @brief 解析替换位置，-1 表示当前集合的最后一张。 */
  std::size_t replaceIndex(const ImageContent & content) const;

  std::string operation_;
  std::int64_t replace_index_{-1};
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__TOOLS__GET_LATEST_COLOR_IMAGE_HPP_
