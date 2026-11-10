#ifndef VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
#include <memory>
#include <optional>
#include <string>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include "vln_behavior_tree/image_content_utils.hpp"
namespace tf2_ros {class Buffer; class TransformListener;}
namespace vln_behavior_tree
{
/** @brief 原生获取最新彩色图，附带采样时刻的小车位姿并写入双向图片端口。 */
class GetLatestColorImageService : public nav2_behavior_tree::BtServiceNode<rgbd_image_queue::srv::GetImage>
{
public:
  using Service = rgbd_image_queue::srv::GetImage;
  using Base = nav2_behavior_tree::BtServiceNode<Service>;
  GetLatestColorImageService(const std::string & name, const BT::NodeConfiguration & config);
  ~GetLatestColorImageService() override;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

protected:
  /** @brief 原生请求固定为 LATEST_COLOR、detail=0。 */
  void on_tick() override;
  BT::NodeStatus on_completion(Service::Response::SharedPtr response) override;

private:
  /** @brief 按图片 ROS 时间查询 map 到 base_link；零时间取最新 TF。 */
  std::optional<Pose2D> lookupPose(const rclcpp::Time & stamp, bool simulated);
  void resetRequest();
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ImageOperation operation_;
};
}  // namespace vln_behavior_tree
#endif  // VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
