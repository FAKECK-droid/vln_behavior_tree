#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include <tf2_ros/buffer.h>
#include <vln_agent/utils/base64.hpp>
#include <vln_agent/utils/json.hpp>

#include "vln_behavior_tree/event_node_base.hpp"
#include "vln_behavior_tree/plugins/events/navigation_event.hpp"

namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::PoseStamped;
using Service = rgbd_image_queue::srv::GetImage;

/** @brief 导航叶节点的本地替身，保留一段 RUNNING 过程并在完成时更新模拟 TF。 */
class FollowPathStub : public BT::StatefulActionNode
{
public:
  using Callback = std::function<BT::NodeStatus(const nav_msgs::msg::Path &, const std::string &)>;
  FollowPathStub(const std::string & name, const BT::NodeConfiguration & config, Callback callback)
  : BT::StatefulActionNode(name, config), callback_(std::move(callback)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("controller_id")};
  }

private:
  BT::NodeStatus onStart() override
  {
    path_ = getInput<nav_msgs::msg::Path>("path").value();
    callback_(path_, "follow_begin");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return callback_(path_, "follow_done");
  }
  void onHalted() override {callback_(path_, "halted");}
  Callback callback_;
  nav_msgs::msg::Path path_;
};

class LookAroundTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    host = std::make_shared<rclcpp::Node>("lookaround_test");
    camera = std::make_shared<rclcpp::Node>("lookaround_camera_stub");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", host);
    blackboard->set("bt_loop_duration", 10ms);
    blackboard->set("server_timeout", 20ms);
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(2000));
    blackboard->set("number_recoveries", 0);
    Pose goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 3;
    goal.pose.position.y = -4;
    goal.pose.orientation.z = std::sin(initial_yaw / 2);
    goal.pose.orientation.w = std::cos(initial_yaw / 2);
    blackboard->set("goal", goal);
    buffer = std::make_shared<tf2_ros::Buffer>(host->get_clock());
    blackboard->set("tf_buffer", buffer);
    updateTf(goal);
    subscription = host->create_subscription<std_msgs::msg::String>(
      "/vln/robot_event", 100, [this](std_msgs::msg::String::ConstSharedPtr value) {
        events.push_back(Json::parse(value->data));
      });
    service = camera->create_service<Service>(
      "/rgbd_image_queue/get_image",
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        int index;
        {
          std::lock_guard<std::mutex> lock(mutex);
          trace.push_back("capture");
          index = static_cast<int>(captures.size());
          captures.push_back(blackboard->get<Pose>("goal"));
          requests.push_back(*request);
        }
        if (index == fail_capture) {
          response->success = false;
          response->message = "image unavailable";
          return;
        }
        response->success = true;
        response->has_color = true;
        response->color.header.frame_id = "camera_optical_frame";
        response->color.header.stamp = camera->now();
        response->color.format = "png";
        response->pair_id = index + 1;
        cv::imencode(
          ".png", cv::Mat(
            180, 320, CV_8UC3,
            cv::Scalar(40 + index * 10, 90, 180)), response->color.data);
      });
    executor.add_node(camera);
    spinner = std::thread([this] {executor.spin();});
    factory.registerFromPlugin(VLN_LOOK_NATIVE_PLUGIN);
    factory.registerFromPlugin(VLN_LOOK_UPDATE_PLUGIN);
    factory.registerFromPlugin(VLN_LOOK_MOSAIC_PLUGIN);
    factory.registerFromPlugin(VLN_LOOK_NAVIGATION_EVENT_PLUGIN);
    factory.registerFromPlugin(VLN_TEST_RECOVERY_PLUGIN);
    factory.registerFromPlugin(VLN_TEST_PIPELINE_PLUGIN);
    factory.registerFromPlugin(VLN_TEST_RATE_PLUGIN);
    factory.registerFromPlugin(VLN_TEST_ROUND_ROBIN_PLUGIN);
    registerNavigationStubs();
  }
  void TearDown() override {executor.cancel(); spinner.join();}
  void registerNavigationStubs()
  {
    factory.registerSimpleAction(
      "ComputePathToPose", [this](BT::TreeNode & node) {
        const auto goal = node.getInput<Pose>("goal").value();
        plans.push_back(goal);
        addTrace("plan");
        if (consumeFailure(planning_failures)) {
          node.setOutput("path", nav_msgs::msg::Path{});
          return BT::NodeStatus::FAILURE;
        }
        nav_msgs::msg::Path path;
        path.header.frame_id = goal.header.frame_id;
        path.poses.push_back(goal);
        node.setOutput("path", path);
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<Pose>("goal"), BT::OutputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<std::string>("planner_id")});
    factory.registerBuilder<FollowPathStub>(
      "FollowPath", [this](
        const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<FollowPathStub>(
          name, config,
          [this](const nav_msgs::msg::Path & path, const std::string & stage) {
            if (stage == "follow_done" && consumeFailure(follow_failures)) {
              addTrace("follow_failed");
              return BT::NodeStatus::FAILURE;
            }
            addTrace(stage);
            if (stage == "follow_begin") {++follow_started;}
            if (stage == "follow_done") {++follow_completed; updateTf(path.poses.back());}
            if (stage == "halted") {++follow_halted;}
            return BT::NodeStatus::SUCCESS;
          });
      });
    const auto recover = [this](BT::TreeNode &) {
        blackboard->set("number_recoveries", blackboard->get<int>("number_recoveries") + 1);
        return recovery_fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      };
    factory.registerSimpleAction(
      "ClearEntireCostmap", recover, {BT::InputPort<std::string>("service_name")});
    factory.registerSimpleCondition(
      "GoalUpdated", [](BT::TreeNode &) {
        return BT::NodeStatus::FAILURE;
      });
    factory.registerSimpleAction(
      "Spin", recover,
      {BT::InputPort<double>("spin_dist")});
    factory.registerSimpleAction(
      "Wait", recover,
      {BT::InputPort<double>("wait_duration")});
    factory.registerSimpleAction(
      "BackUp", recover,
      {BT::InputPort<double>("backup_dist"), BT::InputPort<double>("backup_speed")});
    factory.registerSimpleAction(
      "RemovePassedGoals", [](BT::TreeNode & node) {
        node.setOutput("output_goals", node.getInput<std::vector<Pose>>("input_goals").value());
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::vector<Pose>>("input_goals"),
          BT::OutputPort<std::vector<Pose>>("output_goals"), BT::InputPort<double>("radius")});
    factory.registerSimpleAction(
      "ComputePathThroughPoses", [this](BT::TreeNode & node) {
        const auto goals = node.getInput<std::vector<Pose>>("goals").value();
        plans.push_back(goals.back());
        addTrace("plan");
        if (consumeFailure(planning_failures)) {
          node.setOutput("path", nav_msgs::msg::Path{});
          return BT::NodeStatus::FAILURE;
        }
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.poses = goals;
        node.setOutput("path", path);
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::vector<Pose>>("goals"), BT::OutputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<std::string>("planner_id")});
  }
  bool consumeFailure(int & remaining)
  {
    if (remaining == 0) {return false;}
    if (remaining > 0) {--remaining;}
    return true;
  }
  void updateTf(const Pose & pose)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.rotation = pose.pose.orientation;
    buffer->setTransform(transform, "navigation_stub", true);
  }
  void addTrace(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    trace.push_back(value);
  }
  double yaw(const Pose & pose)
  {
    const auto & q = pose.pose.orientation;
    return std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
  }
  void expectAngle(double actual, double expected)
  {
    EXPECT_NEAR(std::atan2(std::sin(actual - expected), std::cos(actual - expected)), 0, 1e-12);
  }
  BT::Tree lookaround()
  {
    std::ifstream input(VLN_LOOKAROUND_XML);
    const std::string xml((std::istreambuf_iterator<char>(input)),
      std::istreambuf_iterator<char>());
    // 与 Humble 的 BtActionServer 相同，从文本加载；include 由 ros_pkg 定位。
    return factory.createTreeFromText(xml, blackboard);
  }
  BT::NodeStatus finish(BT::Tree & tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto status = tree.tickRoot();
      rclcpp::spin_some(host);
      if (status != BT::NodeStatus::RUNNING) {return status;}
      std::this_thread::sleep_for(2ms);
    }
    return BT::NodeStatus::RUNNING;
  }
  bool awaitEvent(const std::string & type = "lookaround_completed", std::size_t count = 1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(host);
      if (eventCount(type) >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }
  Json payload(const Json & message)
  {
    return Json::parse(message.at("content").at(0).at("text").get<std::string>());
  }
  std::size_t eventCount(const std::string & type)
  {
    std::size_t count = 0;
    for (const auto & message : events) {
      if (payload(message).at("event") == type) {++count;}
    }
    return count;
  }
  Json findEvent(const std::string & type)
  {
    for (const auto & message : events) {
      if (payload(message).at("event") == type) {return message;}
    }
    return Json();
  }
  void expectNavigationPayloads()
  {
    for (const auto & message : events) {
      const auto report = payload(message);
      if (report.at("event").get<std::string>().find("navigation_") != 0) {continue;}
      EXPECT_EQ(message.at("content").size(), 1);
      EXPECT_FALSE(report.at("data").contains("path"));
    }
  }
  BT::Tree navigation(bool multiple)
  {
    if (multiple) {
      auto first = blackboard->get<Pose>("goal");
      auto last = first;
      last.pose.position.x = 8;
      blackboard->set("goals", std::vector<Pose>{first, last});
    }
    return factory.createTreeFromFile(
      multiple ? VLN_MULTIGOAL_XML : VLN_SINGLEGOAL_XML,
      blackboard);
  }
  rclcpp::Node::SharedPtr host, camera;
  rclcpp::Service<Service>::SharedPtr service;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::mutex mutex;
  std::vector<Pose> captures, plans;
  std::vector<Service::Request> requests;
  std::vector<std::string> trace;
  std::vector<Json> events;
  int fail_capture{-1}, follow_started{0}, follow_completed{0}, follow_halted{0};
  int planning_failures{0}, follow_failures{0};
  bool recovery_fails{false};
  const double initial_yaw{0.3};
};

TEST_F(LookAroundTest, RepeatsCaptureUpdateAndNavigationEightTimesThenReportsTfYaw)
{
  ImageContent old;
  old.images.push_back({{1}, {}, "image/png"});
  blackboard->set("image_content", old);
  auto tree = lookaround();
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  ASSERT_EQ(captures.size(), 8);
  ASSERT_EQ(plans.size(), 8);
  EXPECT_EQ(follow_started, 8);
  EXPECT_EQ(follow_completed, 8);
  EXPECT_EQ(blackboard->get<ImageContent>("image_content").images.size(), 8);
  const double step = std::acos(-1) / 4;
  ASSERT_EQ(trace.size(), 32);
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(trace.at(i * 4), "capture");
    EXPECT_EQ(trace.at(i * 4 + 1), "plan");
    EXPECT_EQ(trace.at(i * 4 + 2), "follow_begin");
    EXPECT_EQ(trace.at(i * 4 + 3), "follow_done");
    expectAngle(yaw(captures.at(i)), initial_yaw + i * step);
    expectAngle(yaw(plans.at(i)), initial_yaw + (i + 1) * step);
    EXPECT_EQ(plans.at(i).pose.position.x, 3);
    EXPECT_EQ(plans.at(i).pose.position.y, -4);
    EXPECT_EQ(requests.at(i).operation, Service::Request::LATEST_COLOR);
  }
  ASSERT_EQ(eventCount("navigation_started"), 8);
  ASSERT_EQ(eventCount("navigation_completed"), 8);
  const auto mosaic = findEvent("lookaround_completed");
  EXPECT_EQ(mosaic.at("content").size(), 2);
  const auto report = payload(mosaic);
  expectNavigationPayloads();
  EXPECT_EQ(report.at("event"), "lookaround_completed");
  EXPECT_EQ(report.at("images").size(), 1);
  EXPECT_EQ(report.at("images").at(0).at("width"), 1560);
  EXPECT_EQ(report.at("images").at(0).at("height"), 1560);
  EXPECT_EQ(report.at("data").at("views").size(), 8);
  expectAngle(report.at("pose").at("yaw").get<double>(), initial_yaw + 8 * step);
  EXPECT_EQ(report.at("data").at("yaw"), report.at("pose").at("yaw"));
  for (std::size_t i = 0; i < 8; ++i) {
    const auto & view = report.at("data").at("views").at(i);
    EXPECT_EQ(view.at("pair_id"), i + 1);
    ASSERT_TRUE(view.contains("pose"));
    EXPECT_EQ(view.at("pose").at("frame_id"), "map");
    EXPECT_EQ(view.at("pose").at("x"), 3);
    EXPECT_EQ(view.at("pose").at("y"), -4);
    expectAngle(view.at("pose").at("yaw").get<double>(), initial_yaw + i * step);
  }
}

TEST_F(LookAroundTest, CaptureFailureStopsBeforeNextTurnAndDoesNotPublishIncompleteMosaic)
{
  fail_capture = 2;
  auto tree = lookaround();
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(captures.size(), 3);
  EXPECT_EQ(plans.size(), 2);
  EXPECT_EQ(follow_completed, 2);
  EXPECT_EQ(blackboard->get<ImageContent>("image_content").images.size(), 2);
  ASSERT_TRUE(awaitEvent("navigation_completed", 2));
  EXPECT_EQ(eventCount("navigation_started"), 2);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  EXPECT_EQ(eventCount("lookaround_completed"), 0);
}

TEST_F(LookAroundTest, HaltingRunningSubtreeKeepsStartEventWithoutFalseTerminalEvent)
{
  auto tree = lookaround();
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (follow_started == 0 && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(follow_started, 1);
  tree.haltTree();
  EXPECT_EQ(follow_halted, 1);
  EXPECT_EQ(captures.size(), 1);
  EXPECT_EQ(plans.size(), 1);
  expectAngle(yaw(blackboard->get<Pose>("goal")), initial_yaw + std::acos(-1) / 4);
  ASSERT_TRUE(awaitEvent("navigation_started"));
  EXPECT_EQ(eventCount("navigation_started"), 1);
  EXPECT_EQ(eventCount("navigation_completed"), 0);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  EXPECT_EQ(eventCount("lookaround_completed"), 0);
}

TEST_F(LookAroundTest, NamedMultiGoalTreeLoadsIndependently)
{
  const auto first = blackboard->get<Pose>("goal");
  auto last = first;
  last.pose.position.x = 8;
  blackboard->set("goals", std::vector<Pose>{first, last});
  auto tree = factory.createTreeFromFile(VLN_MULTIGOAL_XML, blackboard);
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(follow_completed, 1);
  EXPECT_EQ(blackboard->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x, 8);
}

class NavigationTreesTest : public LookAroundTest, public ::testing::WithParamInterface<bool>
{
public:
  using LookAroundTest::SetUpTestSuite;
  using LookAroundTest::TearDownTestSuite;
};

TEST_P(NavigationTreesTest, ReportsStartAndCompletionOnceAcrossRunningTicksWithoutPath)
{
  auto tree = navigation(GetParam());
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent("navigation_started"));
  ASSERT_TRUE(awaitEvent("navigation_completed"));
  EXPECT_EQ(eventCount("navigation_started"), 1);
  EXPECT_EQ(eventCount("navigation_completed"), 1);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  EXPECT_EQ(eventCount("navigation_recovery"), 0);
  expectNavigationPayloads();
}

TEST_P(NavigationTreesTest, PlanningRecoveryReportsOnceThenSuccessfulRetryCompletes)
{
  planning_failures = 1;
  auto tree = navigation(GetParam());
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent("navigation_completed"));
  ASSERT_TRUE(awaitEvent("navigation_recovery"));
  EXPECT_EQ(eventCount("navigation_started"), 1);
  EXPECT_EQ(eventCount("navigation_completed"), 1);
  EXPECT_EQ(eventCount("navigation_recovery"), 1);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  EXPECT_NE(
    payload(findEvent("navigation_recovery")).at("origin").get<std::string>().find(
      "PlanningRecovery"), std::string::npos);
  EXPECT_EQ(plans.size(), 2);
  expectNavigationPayloads();
}

TEST_P(NavigationTreesTest, ControllerRecoveryReportsOnceThenSuccessfulRetryCompletes)
{
  follow_failures = 1;
  auto tree = navigation(GetParam());
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent("navigation_completed"));
  ASSERT_TRUE(awaitEvent("navigation_recovery"));
  EXPECT_EQ(eventCount("navigation_recovery"), 1);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  EXPECT_NE(
    payload(findEvent("navigation_recovery")).at("origin").get<std::string>().find(
      "ControlRecovery"), std::string::npos);
  EXPECT_EQ(follow_started, 2);
  EXPECT_EQ(follow_completed, 1);
  expectNavigationPayloads();
}

TEST_P(NavigationTreesTest, SystemRecoveryAfterContextualRetryExhaustionIsReported)
{
  planning_failures = 2;
  auto tree = navigation(GetParam());
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent("navigation_completed"));
  ASSERT_TRUE(awaitEvent("navigation_recovery", 2));
  EXPECT_EQ(eventCount("navigation_recovery"), 2);
  bool found_system_recovery = false;
  for (const auto & message : events) {
    found_system_recovery |=
      payload(message).at("origin").get<std::string>().find("SystemRecovery") !=
      std::string::npos;
  }
  EXPECT_TRUE(found_system_recovery);
  EXPECT_EQ(eventCount("navigation_failed"), 0);
  expectNavigationPayloads();
}

TEST_P(NavigationTreesTest, ExhaustedNavigationReportsFinalFailureAndKeepsFailureStatus)
{
  planning_failures = -1;
  auto tree = navigation(GetParam());
  ASSERT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(awaitEvent("navigation_failed"));
  EXPECT_EQ(eventCount("navigation_started"), 1);
  EXPECT_EQ(eventCount("navigation_failed"), 1);
  EXPECT_EQ(eventCount("navigation_completed"), 0);
  EXPECT_GT(eventCount("navigation_recovery"), 0);
  expectNavigationPayloads();
}

TEST_P(NavigationTreesTest, ReportingFailurePreservesBothNavigationSuccessAndFailure)
{
  ASSERT_TRUE(factory.unregisterBuilder("NavigationEvent"));
  factory.registerSimpleAction(
    "NavigationEvent", [](BT::TreeNode &) {
      return BT::NodeStatus::FAILURE;
    }, NavigationEvent::providedPorts());
  auto successful = navigation(GetParam());
  EXPECT_EQ(finish(successful), BT::NodeStatus::SUCCESS);
  planning_failures = -1;
  recovery_fails = true;
  auto failed = navigation(GetParam());
  EXPECT_EQ(finish(failed), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(events.empty());
}

INSTANTIATE_TEST_SUITE_P(
  DefaultTrees, NavigationTreesTest, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) {return info.param ? "ThroughGoals" : "ToPose";});

}  // namespace vln_behavior_tree::test
