#ifndef VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_
#define VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace vln_behavior_tree
{
/** @brief 通过黑板传递的图片集合；提供方填写图片字节和自定义元数据。 */
struct ImageContent
{
  using MetadataValue = std::variant<std::string, bool, std::int64_t, double>;
  using Metadata = std::map<std::string, MetadataValue>;

  struct Image
  {
    std::vector<unsigned char> bytes;  ///< JPEG、PNG 等图片文件的二进制字节。
    Metadata metadata;
    std::string mime_type{"image/jpeg"};
  };

  std::vector<Image> images;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_
