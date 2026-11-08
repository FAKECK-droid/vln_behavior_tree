#ifndef VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDING_IMAGES_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDING_IMAGES_HPP_
#include "vln_behavior_tree/event_node_base.hpp"
namespace vln_behavior_tree
{
/** @brief 将八张周围图像缩小排列在外圈，中央显示 yaw，合成为一张图片事件。 */
class SubmitSurroundingImages : public EventNodeBase
{
public:
  using EventNodeBase::EventNodeBase;
  static BT::PortsList providedPorts();

protected:
  EventPayload buildPayload() override;
};
}  // namespace vln_behavior_tree
#endif  // VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDING_IMAGES_HPP_
