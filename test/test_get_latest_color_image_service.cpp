#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <vln_agent/utils/json.hpp>
#include "vln_behavior_tree/plugins/services/get_latest_color_image.hpp"
#include "vln_behavior_tree/event_node_base.hpp"
namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
using Service = rgbd_image_queue::srv::GetImage;
static_assert(
  std::is_base_of_v<nav2_behavior_tree::BtServiceNode<Service>,
  GetLatestColorImageService>);
class NativeImageTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    host = std::make_shared<rclcpp::Node>("native_image_bt_test");
    server = std::make_shared<rclcpp::Node>("native_image_service_test");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", host);
    blackboard->set("bt_loop_duration", 10ms);
    blackboard->set("server_timeout", 20ms);
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(2000));
    response.success = true;
    response.has_color = true;
    response.color.header.frame_id = "camera_optical_frame";
    response.color.header.stamp.sec = 1794096000;
    response.color.format = "png";
    cv::imencode(".png", cv::Mat(48, 64, CV_8UC3, cv::Scalar(30, 60, 150)), response.color.data);
    response.pair_id = 99;
    response.queue_index = -1;
    response.queue_size = 3;
    response.image_age_sec = 0.1;
    service = server->create_service<Service>(
      "/rgbd_image_queue/get_image",
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr reply) {
        Service::Response value;
        {
          std::lock_guard<std::mutex> lock(mutex);
          requests.push_back(*request);
          value = response;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms.load()));
        *reply = std::move(value);
        ++completed;
      });
    executor.add_node(server);
    spinner = std::thread([this] {executor.spin();});
    factory.registerFromPlugin(VLN_NATIVE_IMAGE_PLUGIN_PATH);
    factory.registerFromPlugin(VLN_MOSAIC_PLUGIN_PATH);
  }
  void TearDown() override {executor.cancel(); spinner.join();}
  BT::Tree tree(const std::string & body)
  {
    return factory.createTreeFromText(
      "<root><BehaviorTree ID='Main'>" + body + "</BehaviorTree></root>", blackboard);
  }
  BT::NodeStatus finish(BT::Tree & bt)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto status = bt.tickRoot();
      if (status != BT::NodeStatus::RUNNING) {return status;}
      std::this_thread::sleep_for(2ms);
    }
    return BT::NodeStatus::RUNNING;
  }
  bool waitCompleted(int count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (completed.load() >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }
  std::vector<Service::Request> calls()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return requests;
  }
  ImageContent samples()
  {
    ImageContent content;
    content.images.push_back({{10}, {}, "image/jpeg"});
    content.images.push_back({{20}, {}, "image/jpeg"});
    return content;
  }
  rclcpp::Node::SharedPtr host, server;
  rclcpp::Service<Service>::SharedPtr service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  Service::Response response;
  std::mutex mutex;
  std::vector<Service::Request> requests;
  std::atomic_int delay_ms{0}, completed{0};
};
TEST_F(NativeImageTest, DirectServiceUsesNativeFieldsAndPreservesBytesAndMetadata)
{
  delay_ms = 60;
  auto bt = tree("<GetLatestColorImageService image_content='{pictures}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(calls().size(), 1);
  EXPECT_EQ(calls().at(0).operation, Service::Request::LATEST_COLOR);
  EXPECT_EQ(calls().at(0).detail, 0);
  const auto content = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(content.images.size(), 1);
  const auto & image = content.images.at(0);
  EXPECT_EQ(image.bytes, response.color.data);
  EXPECT_EQ(image.mime_type, "image/png");
  EXPECT_FALSE(image.time_stamp.empty());
  EXPECT_EQ(std::get<std::string>(image.metadata.at("frame_id")), "camera_optical_frame");
  EXPECT_EQ(std::get<std::uint64_t>(image.metadata.at("pair_id")), 99);
  EXPECT_EQ(std::get<std::int64_t>(image.metadata.at("queue_index")), -1);
}
TEST_F(NativeImageTest, SupportsSameCollectionOperationsAndClearSubmitsNoRequest)
{
  for (const auto & operation : {"append", "prepend", "replace 0", "replace -1", "clear"}) {
    blackboard->set("pictures", samples());
    blackboard->set("operation", std::string(operation));
    auto bt = tree(
      "<GetLatestColorImageService operation='{operation}' image_content='{pictures}'/>");
    ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS) << operation;
    const auto content = blackboard->get<ImageContent>("pictures");
    const std::string op(operation);
    if (op == "clear") {EXPECT_TRUE(content.images.empty()); continue;}
    const bool replacing = op.find("replace") == 0;
    ASSERT_EQ(content.images.size(), replacing ? 2 : 3);
    const auto index = op == "prepend" || op == "replace 0" ? 0 : content.images.size() - 1;
    EXPECT_EQ(content.images.at(index).bytes, response.color.data);
  }
  EXPECT_EQ(calls().size(), 4);
}
TEST_F(NativeImageTest, InvalidOperationsPreserveCollectionWithoutServiceCalls)
{
  for (const auto & operation :
    {"replace", "replace -2", "replace 2", "append 1", "clear 1", "bad"})
  {
    blackboard->set("pictures", samples());
    blackboard->set("operation", std::string(operation));
    auto bt = tree(
      "<GetLatestColorImageService operation='{operation}' image_content='{pictures}'/>");
    EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
    EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 2);
  }
  blackboard->set("pictures", ImageContent{});
  auto empty = tree(
    "<GetLatestColorImageService operation='replace -1' image_content='{pictures}'/>");
  EXPECT_EQ(finish(empty), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(calls().empty());
}
TEST_F(NativeImageTest, FailureAndEmptyImageLeaveCollectionUntouchedAndCanRecover)
{
  blackboard->set("pictures", samples());
  const auto valid = response;
  response.success = false;
  response.message = "no image";
  auto bt = tree("<GetLatestColorImageService operation='replace 0' image_content='{pictures}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  {
    std::lock_guard<std::mutex> lock(mutex);
    response.success = true;
    response.color.data.clear();
  }
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard->get<ImageContent>("pictures").images.at(0).bytes,
    (std::vector<unsigned char>{10}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    response = valid;
  }
  EXPECT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.at(0).bytes, valid.color.data);
}
TEST_F(NativeImageTest, TimeoutAndHaltAllowFreshRequestWithoutAppendingOldResult)
{
  blackboard->set("pictures", samples());
  delay_ms = 60;
  auto short_wait = tree(
    "<GetLatestColorImageService server_timeout='10' image_content='{pictures}'/>");
  ASSERT_EQ(finish(short_wait), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(waitCompleted(1));
  EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 2);
  auto bt = tree("<GetLatestColorImageService image_content='{pictures}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::RUNNING);
  bt.haltTree();
  ASSERT_TRUE(waitCompleted(2));
  EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 2);
  delay_ms = 0;
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 3);
  EXPECT_EQ(calls().size(), 3);
}
TEST_F(NativeImageTest, SimulatedTimestampIsOmittedAndOperationIsFixedWhileWaiting)
{
  host->set_parameter(rclcpp::Parameter("use_sim_time", true));
  blackboard->set("operation", std::string("append"));
  delay_ms = 60;
  auto bt =
    tree("<GetLatestColorImageService operation='{operation}' image_content='{pictures}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::RUNNING);
  blackboard->set("operation", std::string("clear"));
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto content = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(content.images.size(), 1);
  EXPECT_TRUE(content.images.at(0).time_stamp.empty());
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard->get<ImageContent>("pictures").images.empty());
  EXPECT_EQ(calls().size(), 1);
}
TEST_F(NativeImageTest, EightNativeCapturesFeedOneSurroundingImageEvent)
{
  std::vector<Json> events;
  auto subscription = host->create_subscription<std_msgs::msg::String>(
    "/vln/robot_event", 10, [&](std_msgs::msg::String::ConstSharedPtr value) {
      events.push_back(Json::parse(value->data));
    });
  blackboard->set("robot_pose", Pose2D{"map", 0, 0, 0.7, ""});
  auto bt = tree(
    "<Sequence><Repeat num_cycles='8'><GetLatestColorImageService "
    "image_content='{pictures}'/></Repeat><SubmitSurroundingImages image_content='{pictures}' "
    "yaw='0.7' pose='{robot_pose}'/></Sequence>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (events.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(host);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(events.size(), 1);
  EXPECT_EQ(calls().size(), 8);
  EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 8);
  EXPECT_EQ(events.at(0).at("content").size(), 2);
  const auto payload = Json::parse(events.at(0).at("content").at(0).at("text").get<std::string>());
  EXPECT_EQ(payload.at("images").size(), 1);
  EXPECT_EQ(payload.at("data").at("views").size(), 8);
}
}  // namespace vln_behavior_tree::test
