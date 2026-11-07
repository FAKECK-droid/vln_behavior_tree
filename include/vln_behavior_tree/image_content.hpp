#ifndef VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_
#define VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "vln_behavior_tree/pose_2d.hpp"

namespace vln_behavior_tree
{
/** @brief 通过黑板传递的图片集合；提供方填写图片字节、采样信息和自定义元数据。 */
struct ImageContent
{
  using MetadataValue = std::variant<std::string, bool, std::int64_t, double>;
  using Metadata = std::map<std::string, MetadataValue>;

  struct Image
  {
    std::vector<unsigned char> bytes;       ///< JPEG、PNG 等图片文件的二进制字节。
    Metadata metadata;                     ///< 提供方自定义的图片元数据。
    std::string mime_type{"image/jpeg"};     ///< 图片媒体类型。
    std::string time_stamp{};               ///< 可选的图片采样时间，UTC 三位毫秒字符串。
    std::optional<Pose2D> pose{};            ///< 可选的采图时小车位姿。
  };

  std::vector<Image> images;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__IMAGE_CONTENT_HPP_
