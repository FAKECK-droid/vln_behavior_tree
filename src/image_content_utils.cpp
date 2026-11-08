#include "vln_behavior_tree/image_content_utils.hpp"
#include <mutex>
#include <sstream>
#include <utility>
namespace vln_behavior_tree
{
ImageOperation ImageOperation::parse(const std::string & text)
{
  ImageOperation result;
  std::istringstream parser(text);
  std::string operation, extra;
  if (!(parser >> operation)) {throw BT::RuntimeError("operation 为空");}
  if (operation == "append") {result.type = Type::Append;} else if (operation == "prepend") {
    result.type = Type::Prepend;
  } else if (operation == "replace") {
    result.type = Type::Replace;
    if (!(parser >> result.index)) {throw BT::RuntimeError("请使用 replace <int>");}
  } else if (operation == "clear") {result.type = Type::Clear;} else {
    throw BT::RuntimeError("operation 需要为 append、prepend、replace <int> 或 clear");
  }
  if (parser >> extra) {throw BT::RuntimeError("operation 含有多余参数");}
  return result;
}
std::size_t ImageOperation::replaceIndex(const ImageContent & content) const
{
  if (index < -1 || content.images.empty() ||
    (index >= 0 && static_cast<std::uint64_t>(index) >= content.images.size()))
  {throw BT::RuntimeError("replace 索引超出 image_content 范围");}
  return index == -1 ? content.images.size() - 1 : static_cast<std::size_t>(index);
}
void ImageOperation::validate(const ImageContent & content) const
{
  if (type == Type::Replace) {replaceIndex(content);}
}
void ImageOperation::apply(ImageContent & content, ImageContent::Image image) const
{
  switch (type) {
    case Type::Append: content.images.push_back(std::move(image)); break;
    case Type::Prepend: content.images.insert(content.images.begin(), std::move(image)); break;
    case Type::Replace: content.images.at(replaceIndex(content)) = std::move(image); break;
    case Type::Clear: content.images.clear(); break;
  }
}
ImageContent readImageContent(const BT::TreeNode & node)
{
  const auto & config = node.config();
  const auto port = config.input_ports.find("image_content");
  if (port == config.input_ports.end() || !config.output_ports.count("image_content")) {
    throw BT::RuntimeError("请将 image_content 绑定到黑板变量");
  }
  const auto key = BT::TreeNode::getRemappedKey("image_content", port->second);
  if (!key) {throw BT::RuntimeError("image_content 需要黑板变量");}
  {
    std::unique_lock<std::mutex> lock(config.blackboard->entryMutex());
    const auto value = config.blackboard->getAny(std::string(key.value()));
    if (!value || value->empty()) {return {};}
  }
  const auto content = node.getInput<ImageContent>("image_content");
  if (!content) {throw BT::RuntimeError("image_content 输入无效: ", content.error());}
  return content.value();
}
void writeImageContent(BT::TreeNode & node, const ImageContent & content)
{
  const auto output = node.setOutput("image_content", content);
  if (!output) {throw BT::RuntimeError("image_content 输出失败: ", output.error());}
}
}  // namespace vln_behavior_tree
