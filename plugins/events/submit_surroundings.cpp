#include "vln_behavior_tree/plugins/events/submit_surroundings.hpp"

#include <utility>

namespace vln_behavior_tree
{
SubmitSurroundings::SubmitSurroundings(
  const std::string & name, const BT::NodeConfiguration & config)
: EventNodeBase(name, config)
{
  if (getInput<std::string>("topic")) {preparePublisher();}
}

BT::PortsList SubmitSurroundings::providedPorts()
{
  auto ports = EventNodeBase::providedPorts();
  ports.erase("image_content");
  ports.erase("pose");
  ports.at("event").setDefaultValue("surroundings");
  ports.insert(BT::InputPort<Json>("input", "GetSurroundings 返回的原生 function_call_output"));
  return ports;
}

EventPayload SubmitSurroundings::buildPayload()
{
  auto input = getInput<Json>("input");
  if (!input) {throw BT::RuntimeError("缺少 GetSurroundings 输出: ", input.error());}
  auto & content = input.value().at("output");
  auto payload = Json::parse(content.at(0).at("text").get_ref<const std::string &>());
  if (!payload.at("ok").get<bool>()) {throw BT::RuntimeError("环境图获取失败");}

  auto event = EventNodeBase::buildPayload(ImageContent{}, std::nullopt);
  auto & business = event.business;
  business["data"].update(std::move(payload.at("data")));
  business["pose"] = std::move(payload.at("pose"));
  business["images"] = std::move(payload.at("images"));
  content.erase(content.begin());
  event.image_content = std::move(content);
  return event;
}
}  // namespace vln_behavior_tree
