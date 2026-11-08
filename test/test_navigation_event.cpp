#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <vln_agent/utils/json.hpp>

#include "vln_behavior_tree/plugins/events/navigation_event.hpp"

namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::PoseStamped;

class NavigationEventTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>(
      "navigation_event_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("current_pose", Pose2D{"map", 0, 0, 0.2, ""});
    subscription = node->create_subscription<std_msgs::msg::String>(
      "/vln/robot_event", 10, [this](std_msgs::msg::String::ConstSharedPtr value) {
        events.push_back(Json::parse(value->data));
      });
    factory.registerFromPlugin(VLN_NAVIGATION_EVENT_PLUGIN_PATH);
  }
  BT::Tree tree(const std::string & attributes = "")
  {
    return factory.createTreeFromText(
      "<root><BehaviorTree ID='Main'><NavigationEvent name='ReportNavigation' "
      "pose='{current_pose}' " + attributes + "/></BehaviorTree></root>", blackboard);
  }
  Pose pose(double x, double y, double yaw)
  {
    Pose value;
    value.header.frame_id = "map";
    value.header.stamp.sec = 1794096000;
    value.pose.position.x = x;
    value.pose.position.y = y;
    value.pose.orientation.z = std::sin(yaw / 2);
    value.pose.orientation.w = std::cos(yaw / 2);
    return value;
  }
  bool awaitEvent(std::size_t count = 1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (events.size() >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }
  Json payload(std::size_t i = 0)
  {
    Json accepted = Json::array();
    vln_agent::utils::appendInputs(accepted, events.at(i));
    return Json::parse(accepted.at(0).at("content").at(0).at("text").get<std::string>());
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  std::vector<Json> events;
};

TEST_F(NavigationEventTest, ReadsSingleGoalPathAndRecoveriesFromDefaultBlackboardKeys)
{
  blackboard->set("goal", pose(3, 4, 0.7));
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 1794096000;
  path.poses = {pose(0, 0, 0), pose(3, 4, 0.7)};
  path.poses.at(0).header.frame_id.clear();
  blackboard->set("path", path);
  blackboard->set("number_recoveries", 2);
  auto bt = tree("event='navigation_failed' message='规划失败'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  const auto report = payload();
  EXPECT_EQ(report.at("source"), "robot");
  EXPECT_EQ(report.at("origin"), "ReportNavigation");
  EXPECT_EQ(report.at("event"), "navigation_failed");
  const auto data = report.at("data");
  EXPECT_EQ(data.at("message"), "规划失败");
  EXPECT_EQ(data.at("goal").at("x"), 3);
  EXPECT_NEAR(data.at("goal").at("yaw").get<double>(), 0.7, 1e-12);
  EXPECT_TRUE(data.at("goal").contains("time_stamp"));
  EXPECT_EQ(data.at("path").at("poses").size(), 2);
  EXPECT_EQ(data.at("path").at("poses").at(0).at("frame_id"), "map");
  EXPECT_EQ(data.at("number_recoveries"), 2);
  EXPECT_FALSE(data.contains("goals"));
  EXPECT_EQ(report.at("pose").at("yaw"), 0.2);
}

TEST_F(NavigationEventTest, ReadsMultipleGoalsAndSupportsRemappedInputNames)
{
  blackboard->set("waypoints", std::vector<Pose>{pose(1, 2, 0.3), pose(4, 5, 0.6)});
  blackboard->set("recoveries", 0);
  auto bt = tree("goals='{waypoints}' number_recoveries='{recoveries}'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  const auto data = payload().at("data");
  ASSERT_EQ(data.at("goals").size(), 2);
  EXPECT_EQ(data.at("goals").at(1).at("x"), 4);
  EXPECT_EQ(data.at("number_recoveries"), 0);
  EXPECT_FALSE(data.contains("goal"));
}

TEST_F(NavigationEventTest, MissingOptionalNavigationDataStillPublishesAndFreshInputsAppear)
{
  auto bt = tree("event='navigation_completed'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  EXPECT_TRUE(payload().at("data").empty());
  blackboard->set("goal", pose(7, 8, 0));
  blackboard->set("number_recoveries", 3);
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent(2));
  EXPECT_EQ(payload(1).at("data").at("goal").at("x"), 7);
  EXPECT_EQ(payload(1).at("data").at("number_recoveries"), 3);
}

TEST_F(NavigationEventTest, PreservesImagesAndOmitsUtcSampleStampsInSimulation)
{
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  blackboard->set("goal", pose(3, 4, 0));
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 100;
  path.poses = {pose(3, 4, 0)};
  blackboard->set("path", path);
  ImageContent pictures;
  pictures.images.push_back({{1, 2, 3}, {{"custom_label", std::string("door")}}, "image/png"});
  blackboard->set("pictures", pictures);
  auto bt = tree("image_content='{pictures}'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  const auto data = payload().at("data");
  EXPECT_FALSE(data.at("goal").contains("time_stamp"));
  EXPECT_FALSE(data.at("path").contains("time_stamp"));
  EXPECT_FALSE(data.at("path").at("poses").at(0).contains("time_stamp"));
  EXPECT_EQ(events.at(0).at("content").size(), 2);
  EXPECT_EQ(payload().at("images").at(0).at("custom_label"), "door");
}

TEST_F(NavigationEventTest, IncorrectTypedDataFailsWithoutPublishingAndCanBeCorrected)
{
  blackboard->set("wrong", std::string("invalid pose"));
  auto bt = tree("goal='{wrong}'");
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  rclcpp::spin_some(node);
  EXPECT_TRUE(events.empty());
  auto corrected = tree();
  blackboard->set("goal", pose(3, 4, 0));
  EXPECT_EQ(corrected.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
}
}  // namespace vln_behavior_tree::test
