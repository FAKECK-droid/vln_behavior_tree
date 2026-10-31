#include <chrono>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include "vln_behavior_tree/event_node_base.hpp"

namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;

class ActionEventNode : public EventNodeBase
{
public:
  using EventNodeBase::EventNodeBase;

  static BT::PortsList providedPorts()
  {
    auto ports = EventNodeBase::providedPorts();
    ports.insert(BT::InputPort<std::string>("action"));
    ports.insert(BT::InputPort<double>("distance_remaining_m"));
    return ports;
  }

protected:
  EventPayload buildPayload() override
  {
    auto payload = EventNodeBase::buildPayload();
    payload.business["data"] = {{"action", getInput<std::string>("action").value()},
      {"distance_remaining_m", getInput<double>("distance_remaining_m").value()}};
    return payload;
  }
};

class SnapshotEventNode : public EventNodeBase
{
public:
  using EventNodeBase::EventNodeBase;

protected:
  EventPayload buildPayload() override
  {
    auto event = EventNodeBase::buildPayload();
    config().blackboard->set("pictures", ImageContent{});
    return event;
  }
};

class EventNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>(
      "event_node_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    factory.registerFromPlugin(VLN_EVENT_PLUGIN_PATH);
    executor.add_node(node);
  }

  BT::Tree tree(const std::string & body)
  {
    return factory.createTreeFromText(
      "<root main_tree_to_execute='MainTree'><BehaviorTree ID='MainTree'>" +
      body + "</BehaviorTree></root>", blackboard);
  }

  void subscribe(const std::string & topic = "/vln/robot_event")
  {
    subscriptions.push_back(
      node->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          received.push_back(Json::parse(message->data));
        }));
  }

  bool awaitMessages(std::size_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (received.size() >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  Json payload(std::size_t index = 0) const
  {
    return Json::parse(received.at(index).at("content").at(0).at("text").get<std::string>());
  }

  std::shared_ptr<rclcpp::Node> node;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
  std::vector<Json> received;
};

TEST_F(EventNodeTest, PluginPublishesNativeEventWithFreshInputs)
{
  subscribe();
  blackboard->set("kind", std::string("error"));
  blackboard->set("description", std::string("导航执行失败。"));
  auto bt = tree(
    "<SubmitEvent name='ReportNavigationFailure' event='{kind}' text='{description}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  EXPECT_EQ(received[0].at("role"), "user");
  EXPECT_EQ(received[0].at("content").size(), 1);
  EXPECT_EQ(received[0].at("content").at(0).at("type"), "input_text");
  const auto first = payload();
  EXPECT_EQ(first.at("source"), "robot");
  EXPECT_EQ(first.at("origin"), "ReportNavigationFailure");
  EXPECT_EQ(first.at("event"), "error");
  EXPECT_EQ(first.at("text"), "导航执行失败。");
  EXPECT_TRUE(
    std::regex_match(
      first.at("time_stamp").get<std::string>(),
      std::regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z)")));

  blackboard->set("kind", std::string("status"));
  blackboard->set("description", std::string("正在恢复。"));
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(2));
  EXPECT_EQ(payload(1).at("event"), "status");
  EXPECT_EQ(payload(1).at("text"), "正在恢复。");
}

TEST_F(EventNodeTest, DerivedNodeBuildsBusinessPayloadFromTypedInputs)
{
  subscribe();
  factory.registerNodeType<ActionEventNode>("ReportAction");
  blackboard->set("distance", 2.4);
  auto bt = tree(
    "<ReportAction name='ReportFollowPath' action='FollowPath' "
    "distance_remaining_m='{distance}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  const auto report = payload();
  EXPECT_EQ(report.at("event"), "status");
  EXPECT_EQ(report.at("origin"), "ReportFollowPath");
  EXPECT_EQ(
    report.at("data"),
    (Json{{"action", "FollowPath"}, {"distance_remaining_m", 2.4}}));
  EXPECT_FALSE(report.contains("text"));
}

TEST_F(EventNodeTest, EncodesImageBytesAndAllCustomMetadataWithoutChangingInput)
{
  subscribe();
  ImageContent pictures;
  pictures.images.push_back(
    {{0, 1, 2},
      {{"label", std::string("door")}, {"sequence", std::int64_t{7}},
        {"score", 0.97}, {"usable", true}, {"image_url", std::string("custom metadata")}},
      "image/jpeg"});
  pictures.images.push_back({{4, 5}, {}, "image/png"});
  blackboard->set("pictures", pictures);
  auto bt = tree("<SubmitEvent image_content='{pictures}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  const auto & content = received[0].at("content");
  ASSERT_EQ(content.size(), 3);
  EXPECT_EQ(
    content.at(1), (Json{{"type", "input_image"},
      {"image_url", "data:image/jpeg;base64,AAEC"}, {"detail", "auto"}}));
  EXPECT_EQ(
    content.at(2), (Json{{"type", "input_image"},
      {"image_url", "data:image/png;base64,BAU="}, {"detail", "auto"}}));
  const auto metadata = payload().at("images");
  ASSERT_EQ(metadata.size(), 2);
  EXPECT_EQ(
    metadata.at(0), (Json{{"label", "door"}, {"sequence", 7}, {"score", 0.97},
      {"usable", true}, {"image_url", "custom metadata"}}));
  EXPECT_TRUE(metadata.at(0).at("sequence").is_number_integer());
  EXPECT_TRUE(metadata.at(0).at("score").is_number_float());
  EXPECT_TRUE(metadata.at(0).at("usable").is_boolean());
  EXPECT_EQ(metadata.at(1), Json::object());
  const auto original = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(original.images.size(), pictures.images.size());
  EXPECT_EQ(original.images.at(0).bytes, pictures.images.at(0).bytes);
  EXPECT_EQ(original.images.at(0).metadata, pictures.images.at(0).metadata);
}

TEST_F(EventNodeTest, ReadsImageInputOnlyWhileBuildingPayload)
{
  subscribe();
  factory.registerNodeType<SnapshotEventNode>("SnapshotEvent");
  ImageContent pictures;
  pictures.images.push_back({{1}, {{"sample", std::int64_t{42}}}, "image/png"});
  blackboard->set("pictures", pictures);
  auto bt = tree("<SnapshotEvent image_content='{pictures}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  EXPECT_TRUE(blackboard->get<ImageContent>("pictures").images.empty());
  EXPECT_EQ(received[0].at("content").size(), 2);
  EXPECT_EQ(received[0].at("content").at(1).at("image_url"), "data:image/png;base64,AQ==");
  EXPECT_EQ(payload().at("images").at(0), (Json{{"sample", 42}}));
}

TEST_F(EventNodeTest, ResolvesUpdatedTopicAndImageInput)
{
  subscribe("/event_node_test/first");
  subscribe("/event_node_test/second");
  blackboard->set("destination", std::string("/event_node_test/first"));
  ImageContent pictures;
  pictures.images.push_back({{1}, {}, "image/png"});
  blackboard->set("pictures", pictures);
  auto bt = tree("<SubmitEvent topic='{destination}' image_content='{pictures}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  EXPECT_EQ(received[0].at("content").size(), 2);
  EXPECT_EQ(node->count_publishers("/event_node_test/first"), 1);
  blackboard->set("destination", std::string("/event_node_test/second"));
  pictures.images.at(0).bytes = {4, 5};
  blackboard->set("pictures", pictures);
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(2));
  EXPECT_EQ(node->count_publishers("/event_node_test/second"), 1);
  EXPECT_EQ(received[1].at("content").at(1).at("image_url"), "data:image/png;base64,BAU=");
}

TEST_F(EventNodeTest, InvalidInputFailsAndCanBeCorrectedOnNextExecution)
{
  subscribe();
  ImageContent pictures;
  pictures.images.push_back({{}, {}, "image/png"});
  blackboard->set("pictures", pictures);
  auto bt = tree("<SubmitEvent image_content='{pictures}'/>");
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(received.empty());
  blackboard->set("pictures", ImageContent{});
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitMessages(1));
  EXPECT_FALSE(payload().contains("images"));

  auto missing_images = tree("<SubmitEvent image_content='{missing_pictures}'/>");
  EXPECT_EQ(missing_images.tickRoot(), BT::NodeStatus::FAILURE);
  auto bad_topic = tree("<SubmitEvent topic='invalid topic'/>");
  EXPECT_EQ(bad_topic.tickRoot(), BT::NodeStatus::FAILURE);
  auto missing_input = tree("<SubmitEvent text='{missing_description}'/>");
  EXPECT_EQ(missing_input.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(EventNodeTest, NodeDoesNotRetainHostAfterBlackboardReleasesIt)
{
  auto host = std::make_shared<rclcpp::Node>("temporary_event_host");
  const std::weak_ptr<rclcpp::Node> weak_host = host;
  blackboard->set("node", host);
  auto bt = tree("<SubmitEvent/>");
  blackboard->set("node", node);
  host.reset();
  EXPECT_TRUE(weak_host.expired());
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
}
}  // namespace vln_behavior_tree::test
