#include "vln_behavior_tree/plugins/events/submit_surrounding_images.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <utility>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <vln_agent/utils/json.hpp>
#include <vln_agent/utils/time.hpp>
namespace vln_behavior_tree
{
namespace
{
template<typename T>
T input(const BT::TreeNode & node, const std::string & port)
{
  const auto value = node.getInput<T>(port);
  if (!value) {throw BT::RuntimeError("环视事件输入 '", port, "' 无效: ", value.error());}
  return value.value();
}
/** @brief 在当前网格内部居中绘制文字，按网格宽度调整字体。 */
void putCentered(cv::Mat & tile, const std::string & text, int y)
{
  int baseline = 0;
  const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 1, &baseline);
  const double scale = std::min(0.7, tile.cols * 0.9 / std::max(1, size.width));
  const auto fitted = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
  cv::putText(
    tile, text, cv::Point((tile.cols - fitted.width) / 2, y),
    cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
}
/** @brief 保存与事件附件相同的 JPEG 字节，日志写入失败只影响诊断。 */
void saveToLog(const std::vector<unsigned char> & bytes, const rclcpp::Logger & logger)
{
  try {
    static std::atomic<std::uint64_t> sequence{0};
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto directory = std::filesystem::path(rclcpp::get_logging_directory().string()) /
      "vln_behavior_tree" / "surrounding_images";
    std::filesystem::create_directories(directory);
    const auto path = directory /
      ("surrounding_" + std::to_string(stamp) + "_" + std::to_string(++sequence) + ".jpg");
    std::ofstream file;
    file.exceptions(std::ios::failbit | std::ios::badbit);
    file.open(path, std::ios::binary);
    file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    file.close();
    RCLCPP_INFO(logger, "环视合并图已保存: %s", path.string().c_str());
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger, "环视合并图保存失败: %s", error.what());
  }
}
}
BT::PortsList SubmitSurroundingImages::providedPorts()
{
  auto ports = EventNodeBase::providedPorts();
  ports.insert(
    BT::InputPort<std::string>(
      "direction", "counterclockwise", "clockwise 或 counterclockwise，从正上方开始排列"));
  ports.insert(BT::InputPort<int>("tile_size", 256, "每格边长，单位像素"));
  ports.insert(BT::InputPort<int>("gap_size", 12, "相邻网格之间的间隔，单位像素"));
  ports.insert(BT::InputPort<bool>("save_to_log", false, "将提交的合并图保存到 ROS 日志目录"));
  return ports;
}
EventPayload SubmitSurroundingImages::buildPayload()
{
  const auto pictures = input<ImageContent>(*this, "image_content");
  if (pictures.images.size() != 8) {throw BT::RuntimeError("环视事件需要八张图片");}
  const auto pose = readPose();
  const auto direction = input<std::string>(*this, "direction");
  const auto tile_size = input<int>(*this, "tile_size");
  const auto gap_size = input<int>(*this, "gap_size");
  const auto save_to_log = input<bool>(*this, "save_to_log");
  if (pose && !std::isfinite(pose->yaw)) {throw BT::RuntimeError("当前位姿的 yaw 需要为有限数值");}
  if (direction != "clockwise" && direction != "counterclockwise") {
    throw BT::RuntimeError("direction 需要为 clockwise 或 counterclockwise");
  }
  if (tile_size <= 0 || tile_size > std::numeric_limits<int>::max() / 3) {
    throw BT::RuntimeError("tile_size 需要为有效的正像素尺寸");
  }
  const auto side = std::int64_t{3} *tile_size + std::int64_t{2} *gap_size;
  if (gap_size < 0 || side > std::numeric_limits<int>::max()) {
    throw BT::RuntimeError("gap_size 需要为有效的非负像素尺寸");
  }
  const int stride = tile_size + gap_size;
  cv::Mat mosaic(static_cast<int>(side), static_cast<int>(side), CV_8UC3,
    cv::Scalar(180, 180, 180));
  // 坐标为列、行；从正上方沿顺时针枚举八个外圈网格。
  const std::array<cv::Point, 8> ring{{{1, 0}, {2, 0}, {2, 1}, {2, 2},
    {1, 2}, {0, 2}, {0, 1}, {0, 0}}};
  Json views = Json::array();
  for (std::size_t i = 0; i < pictures.images.size(); ++i) {
    const auto & image = pictures.images.at(i);
    if (image.bytes.empty()) {throw BT::RuntimeError("环视图片字节为空，索引: ", std::to_string(i));}
    const auto decoded = cv::imdecode(image.bytes, cv::IMREAD_COLOR);
    if (decoded.empty()) {throw BT::RuntimeError("环视图片解码失败，索引: ", std::to_string(i));}
    const auto cell = ring.at(direction == "clockwise" ? i : (8 - i) % 8);
    auto tile = mosaic(cv::Rect(cell.x * stride, cell.y * stride, tile_size, tile_size));
    tile.setTo(cv::Scalar(12, 12, 12));
    const double scale = std::min(
      {1.0, double(tile_size) / decoded.cols,
        double(tile_size) / decoded.rows});
    cv::Mat resized;
    cv::resize(
      decoded, resized, cv::Size(
        std::max(1, int(std::lround(decoded.cols * scale))),
        std::max(1, int(std::lround(decoded.rows * scale)))), 0, 0, cv::INTER_AREA);
    resized.copyTo(
      tile(
        cv::Rect(
          (tile_size - resized.cols) / 2, (tile_size - resized.rows) / 2,
          resized.cols, resized.rows)));
    cv::putText(
      tile, std::to_string(i + 1), cv::Point(4, std::max(12, tile_size / 12)),
      cv::FONT_HERSHEY_SIMPLEX, std::min(0.6, tile_size / 200.0), cv::Scalar(255, 255, 255),
      1, cv::LINE_AA);
    Json description = Json::object();
    for (const auto & field : image.metadata) {
      std::visit([&](const auto & value) {description[field.first] = value;}, field.second);
    }
    if (!image.time_stamp.empty()) {description["time_stamp"] = image.time_stamp;}
    if (image.pose) {
      const auto & pose = *image.pose;
      description["pose"] = vln_agent::utils::posePayload(
        pose.frame_id, pose.x, pose.y, pose.yaw, pose.time_stamp);
    }
    description["source_index"] = i;
    description["row"] = cell.y;
    description["column"] = cell.x;
    views.push_back(std::move(description));
  }
  auto center = mosaic(cv::Rect(stride, stride, tile_size, tile_size));
  center.setTo(cv::Scalar(32, 32, 32));
  constexpr double pi = 3.14159265358979323846;
  putCentered(center, pose ? "yaw (" + pose->frame_id + ")" : "yaw", tile_size * 2 / 5);
  if (pose) {
    putCentered(center, cv::format("%.1f deg", pose->yaw * 180.0 / pi), tile_size * 3 / 5);
  } else {
    putCentered(center, "unavailable", tile_size * 3 / 5);
  }
  ImageContent output;
  ImageContent::Image composite;
  composite.mime_type = "image/jpeg";
  composite.time_stamp = vln_agent::utils::utcNow();
  composite.metadata = {{"layout", std::string("surround_3x3")}, {"direction", direction},
    {"view_count", std::int64_t{8}}, {"tile_size", std::int64_t{tile_size}},
    {"gap_size", std::int64_t{gap_size}}, {"width", std::int64_t{mosaic.cols}},
    {"height", std::int64_t{mosaic.rows}}};
  if (!cv::imencode(".jpg", mosaic, composite.bytes, {cv::IMWRITE_JPEG_QUALITY, 85})) {
    throw BT::RuntimeError("环视图片 JPEG 编码失败");
  }
  output.images.push_back(std::move(composite));
  auto event = EventNodeBase::buildPayload(output, pose);
  if (pose) {event.business["data"]["yaw"] = pose->yaw;}
  event.business["data"]["views"] = std::move(views);
  if (save_to_log) {
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    saveToLog(output.images.front().bytes, node->get_logger());
  }
  return event;
}
}  // namespace vln_behavior_tree
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::SubmitSurroundingImages>("SubmitSurroundingImages");
}
