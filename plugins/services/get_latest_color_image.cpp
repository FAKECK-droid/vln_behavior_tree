#include "vln_behavior_tree/plugins/services/get_latest_color_image.hpp"
#include <chrono>
#include <exception>
#include <utility>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <vln_agent/utils/time.hpp>
namespace vln_behavior_tree
{
GetLatestColorImageService::GetLatestColorImageService(
  const std::string & name, const BT::NodeConfiguration & config)
: Base(name, config, "/rgbd_image_queue/get_image") {}
BT::PortsList GetLatestColorImageService::providedPorts()
{
  auto ports = Base::providedBasicPorts(
    {
      BT::InputPort<std::string>("operation", "append", "append、prepend、replace <int> 或 clear"),
      BT::BidirectionalPort<ImageContent>("image_content", "读取并更新的图片集合")});
  ports.at("service_name") = BT::InputPort<std::string>(
    "service_name", "/rgbd_image_queue/get_image", "原生 GetImage 服务").second;
  ports.at("server_timeout").setDefaultValue("5000");
  ports.at("server_timeout").setDescription("等待图片结果的超时，单位毫秒");
  return ports;
}
void GetLatestColorImageService::on_tick()
{
  const auto input = getInput<std::string>("operation");
  if (!input) {throw BT::RuntimeError("operation 输入无效: ", input.error());}
  operation_ = ImageOperation::parse(input.value());
  operation_.validate(readImageContent(*this));
  request_->operation = Service::Request::LATEST_COLOR;
  request_->detail = 0;
}
BT::NodeStatus GetLatestColorImageService::on_completion(Service::Response::SharedPtr response)
{
  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(), "获取彩色图失败: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (response->color.data.empty()) {throw BT::RuntimeError("取图结果的图片字节为空");}
  const auto & color = response->color;
  ImageContent::Image image;
  image.bytes = color.data;
  image.mime_type = color.format.find("png") != std::string::npos ? "image/png" : "image/jpeg";
  image.metadata = {
    {"frame_id", color.header.frame_id}, {"format", color.format},
    {"pair_id", response->pair_id}, {"queue_index", response->queue_index},
    {"queue_size", std::uint64_t{response->queue_size}}, {"image_age_sec", response->image_age_sec},
    {"age_error_sec", response->age_error_sec}, {"exceeded_max_age", response->exceeded_max_age}};
  bool simulated = false;
  node_->get_parameter_or("use_sim_time", simulated, false);
  const auto & stamp = color.header.stamp;
  if (!simulated && (stamp.sec != 0 || stamp.nanosec != 0)) {
    image.time_stamp = vln_agent::utils::formatUtcTime(
      std::chrono::system_clock::time_point(
        std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
  }
  auto content = readImageContent(*this);
  operation_.apply(content, std::move(image));
  writeImageContent(*this, content);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus GetLatestColorImageService::tick()
{
  try {
    if (!request_sent_) {
      const auto input = getInput<std::string>("operation");
      if (!input) {throw BT::RuntimeError("operation 输入无效: ", input.error());}
      if (ImageOperation::parse(input.value()).type == ImageOperation::Type::Clear) {
        writeImageContent(*this, ImageContent{});
        return BT::NodeStatus::SUCCESS;
      }
    }
    const auto result = Base::tick();
    if (result != BT::NodeStatus::RUNNING) {resetRequest();}
    return result;
  } catch (const std::exception & error) {
    resetRequest();
    RCLCPP_ERROR(node_->get_logger(), "原生图像节点 '%s' 执行失败: %s", name().c_str(), error.what());
    return BT::NodeStatus::FAILURE;
  }
}
void GetLatestColorImageService::resetRequest()
{
  service_client_->prune_pending_requests();
  request_sent_ = false;
  future_result_ = {};
}
void GetLatestColorImageService::halt()
{
  resetRequest();
  Base::halt();
}
}  // namespace vln_behavior_tree
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vln_behavior_tree::GetLatestColorImageService>(
    "GetLatestColorImageService");
}
