#ifndef VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDING_IMAGES_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDING_IMAGES_HPP_
#include "vln_behavior_tree/event_node_base.hpp"
namespace vln_behavior_tree
{
/** @brief 将八张周围图像以间隔排列在外圈，中央显示 yaw，可将合并图保存至日志。 */
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
