#ifndef VLN_BEHAVIOR_TREE__IMAGE_CONTENT_UTILS_HPP_
#define VLN_BEHAVIOR_TREE__IMAGE_CONTENT_UTILS_HPP_
#include <cstdint>
#include <string>
#include <behaviortree_cpp_v3/tree_node.h>
#include "vln_behavior_tree/image_content.hpp"
namespace vln_behavior_tree
{
/** @brief 两种取图方式共用的图片集合操作。 */
struct ImageOperation
{
  enum class Type {Append, Prepend, Replace, Clear};
  Type type{Type::Append};
  std::int64_t index{-1};
  static ImageOperation parse(const std::string & text);
  void validate(const ImageContent & content) const;
  void apply(ImageContent & content, ImageContent::Image image) const;

private:
  std::size_t replaceIndex(const ImageContent & content) const;
};
/** @brief 读取双向图片端口，尚未初始化的黑板变量作为空集合。 */
ImageContent readImageContent(const BT::TreeNode & node);
/** @brief 将图片集合写回双向端口。 */
void writeImageContent(BT::TreeNode & node, const ImageContent & content);
}  // namespace vln_behavior_tree
#endif  // VLN_BEHAVIOR_TREE__IMAGE_CONTENT_UTILS_HPP_
