#ifndef VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDINGS_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDINGS_HPP_

#include "vln_behavior_tree/event_node_base.hpp"

namespace vln_behavior_tree
{
/** @brief 将 GetSurroundings 的业务数据、采样位姿和原生图片作为机器人事件发布。 */
class SubmitSurroundings : public EventNodeBase
{
public:
  SubmitSurroundings(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  EventPayload buildPayload() override;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__SUBMIT_SURROUNDINGS_HPP_
