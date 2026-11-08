#include "vln_behavior_tree/plugins/tools/get_latest_color_image.hpp"

#include <exception>
#include <string_view>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <vln_agent/utils/base64.hpp>

namespace vln_behavior_tree
{
namespace
{
/** @brief 将服务中的原生图片及描述恢复为普通 C++ 图片数据。 */
ImageContent::Image imageFromResult(const vln_agent::ToolResult & result)
{
  if (result.images.size() != 1) {throw BT::RuntimeError("取图工具需要返回一张图片");}
  const auto & part = result.images.at(0);
  if (part.at("type") != "input_image") {throw BT::RuntimeError("取图结果类型需要为 input_image");}
  const auto url = part.at("image_url").get<std::string>();
  const auto separator = url.find(";base64,");
  if (url.compare(0, 5, "data:") != 0 || separator == std::string::npos || separator == 5) {
    throw BT::RuntimeError("取图结果需要为 Base64 图片 data URL");
  }
  ImageContent::Image image;
  image.mime_type = url.substr(5, separator - 5);
  image.bytes = vln_agent::utils::decodeBase64(std::string_view(url).substr(separator + 8));
  if (image.bytes.empty()) {throw BT::RuntimeError("取图结果的图片字节为空");}
  const auto & description = result.payload.at("images").at(0);
  const auto & metadata = description.at("metadata");
  if (!metadata.is_object()) {throw BT::RuntimeError("图片 metadata 需要为对象");}
  for (const auto & field : metadata.items()) {
    const auto & value = field.value();
    if (value.is_string()) {image.metadata[field.key()] = value.get<std::string>();} else if (
      value.is_boolean()) {image.metadata[field.key()] = value.get<bool>();} else if (
      value.is_number_unsigned())
    {
      image.metadata[field.key()] = value.get<std::uint64_t>();
    } else if (
      value.is_number_integer())
    {
      image.metadata[field.key()] = value.get<std::int64_t>();
    } else if (
      value.is_number_float()) {image.metadata[field.key()] = value.get<double>();} else {
      throw BT::RuntimeError("图片元数据值需要为字符串、布尔或数值: ", field.key());
    }
  }
  image.time_stamp = description.value("time_stamp", std::string());
  if (description.contains("pose") && !description.at("pose").is_null()) {
    const auto & pose = description.at("pose");
    image.pose = Pose2D{pose.at("frame_id").get<std::string>(), pose.at("x").get<double>(),
      pose.at("y").get<double>(), pose.at("yaw").get<double>(),
      pose.value("time_stamp", std::string())};
  }
  return image;
}
}  // namespace

BT::PortsList GetLatestColorImage::providedPorts()
{
  auto ports = CallToolNodeBase::providedPorts();
  ports.erase("tool");
  ports.erase("arguments");
  ports.erase("result");
  ports.insert(
    BT::InputPort<std::string>(
      "operation", "append", "append、prepend、replace <int> 或 clear"));
  ports.insert(BT::BidirectionalPort<ImageContent>("image_content", "读取并更新的图片集合"));
  return ports;
}

std::string GetLatestColorImage::toolName() {return "capture_image";}

GetLatestColorImage::Json GetLatestColorImage::buildArguments()
{
  const auto input = getInput<std::string>("operation");
  if (!input) {throw BT::RuntimeError("operation 输入无效: ", input.error());}
  operation_ = ImageOperation::parse(input.value());
  operation_.validate(readImageContent(*this));
  return {{"mode", "latest"}, {"detail", 0}};
}

BT::NodeStatus GetLatestColorImage::onResult(const ToolResult & result)
{
  if (!result.payload.at("ok").get<bool>()) {
    RCLCPP_WARN(node_->get_logger(), "获取彩色图失败: %s", result.payload.dump().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto image = imageFromResult(result);
  auto content = readImageContent(*this);
  operation_.apply(content, std::move(image));
  writeImageContent(*this, content);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetLatestColorImage::tick()
{
  try {
    if (status() != BT::NodeStatus::RUNNING) {
      const auto input = getInput<std::string>("operation");
      if (!input) {throw BT::RuntimeError("operation 输入无效: ", input.error());}
      if (ImageOperation::parse(input.value()).type == ImageOperation::Type::Clear) {
        writeImageContent(*this, ImageContent{});
        return BT::NodeStatus::SUCCESS;
      }
    }
    return CallToolNodeBase::tick();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node_->get_logger(), "图像节点 '%s' 执行失败: %s", name().c_str(), error.what());
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vln_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::GetLatestColorImage>("GetLatestColorImage");
}
