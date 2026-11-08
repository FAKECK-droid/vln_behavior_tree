#ifndef VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
#include <string>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include "vln_behavior_tree/image_content_utils.hpp"
namespace vln_behavior_tree
{
/** @brief 原生调用 GetImage 服务，将最新彩色图写入双向图片端口。 */
class GetLatestColorImageService : public nav2_behavior_tree::BtServiceNode<rgbd_image_queue::srv::GetImage>
{
public:
  using Service = rgbd_image_queue::srv::GetImage;
  using Base = nav2_behavior_tree::BtServiceNode<Service>;
  GetLatestColorImageService(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

protected:
  /** @brief 原生请求固定为 LATEST_COLOR、detail=0。 */
  void on_tick() override;
  BT::NodeStatus on_completion(Service::Response::SharedPtr response) override;

private:
  void resetRequest();
  ImageOperation operation_;
};
}  // namespace vln_behavior_tree
#endif  // VLN_BEHAVIOR_TREE__PLUGINS__SERVICES__GET_LATEST_COLOR_IMAGE_HPP_
