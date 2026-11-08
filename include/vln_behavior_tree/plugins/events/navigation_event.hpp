#ifndef VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__NAVIGATION_EVENT_HPP_
#define VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__NAVIGATION_EVENT_HPP_

#include "vln_behavior_tree/event_node_base.hpp"

namespace vln_behavior_tree
{
/** @brief 使用 Nav2 导航黑板中的目标和恢复次数构建机器事件。 */
class NavigationEvent : public EventNodeBase
{
public:
  using EventNodeBase::EventNodeBase;
  static BT::PortsList providedPorts();

protected:
  EventPayload buildPayload() override;
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__PLUGINS__EVENTS__NAVIGATION_EVENT_HPP_
