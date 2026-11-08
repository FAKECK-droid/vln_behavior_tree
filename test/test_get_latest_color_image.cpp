#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <rgbd_image_queue/srv/get_image.hpp>
#include <vln_agent/tools/tool_manager.hpp>
#include <tf2_ros/buffer.h>
#include <vln_agent/utils/base64.hpp>

#include "vln_behavior_tree/plugins/tools/get_latest_color_image.hpp"
#include "vln_behavior_tree/event_node_base.hpp"

namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
using Service = vln_agent::srv::ToolCall;
using ToolResult = vln_agent::ToolResult;

class ColorImageTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    host = std::make_shared<rclcpp::Node>("color_image_bt_test");
    server = std::make_shared<rclcpp::Node>("color_image_tool_service");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", host);
    blackboard->set("bt_loop_duration", 10ms);
    blackboard->set("server_timeout", 20ms);
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(2000));
    auto buffer = std::make_shared<tf2_ros::Buffer>(host->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 2.5;
    transform.transform.rotation.w = 1.0;
    buffer->setTransform(transform, "test", true);
    blackboard->set("tf_buffer", buffer);
    result = ToolResult::success();
    result.payload["images"] = Json::array(
      {{
        {"time_stamp", "2026-11-08T02:30:00.080Z"},
        {"metadata", {{"custom_label", "door"}, {"pair_id", std::uint64_t{42}},
            {"max_counter", std::numeric_limits<std::uint64_t>::max()},
            {"score", 0.97}, {"usable", true}, {"signed_value", -3}}}}});
    result.images = {{{"type", "input_image"},
      {"image_url", "data:image/png;base64,AAEC/w=="}, {"detail", "auto"}}};
    service = server->create_service<Service>(
      "/vln/tool_call",
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        ToolResult response_result;
        {
          std::lock_guard<std::mutex> lock(mutex);
          requests.push_back(Json::parse(request->request_json));
          response_result = result;
        }
        const auto parsed = Json::parse(request->request_json);
        response->outputs_json = Json::array(
          {vln_agent::utils::functionCallOutput(
              parsed.at("output").at(0).at("call_id").get<std::string>(),
              response_result.payload, response_result.images)}).dump();
      });
    executor.add_node(server);
    spinner = std::thread([this] {executor.spin();});
    factory.registerFromPlugin(VLN_IMAGE_PLUGIN_PATH);
    factory.registerFromPlugin(VLN_EVENT_PLUGIN_PATH);
  }

  void TearDown() override
  {
    executor.cancel();
    spinner.join();
  }

  BT::Tree tree(const std::string & body)
  {
    return factory.createTreeFromText(
      "<root main_tree_to_execute='MainTree'><BehaviorTree ID='MainTree'>" +
      body + "</BehaviorTree></root>", blackboard);
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

  std::vector<Json> calls()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return requests;
  }

  void setResult(ToolResult value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    result = std::move(value);
  }

  ImageContent samples()
  {
    ImageContent value;
    value.images.push_back({{10}, {{"label", std::string("first")}}, "image/jpeg"});
    value.images.push_back({{20}, {{"label", std::string("last")}}, "image/jpeg"});
    return value;
  }

  rclcpp::Node::SharedPtr host, server;
  rclcpp::Service<Service>::SharedPtr service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  std::mutex mutex;
  ToolResult result;
  std::vector<Json> requests;
};

TEST_F(ColorImageTest, AppendsToNewBlackboardVariableAndPreservesImageData)
{
  auto bt = tree("<GetLatestColorImage image_content='{pictures}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  auto pictures = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(pictures.images.size(), 1);
  const auto & image = pictures.images.at(0);
  EXPECT_EQ(image.bytes, (std::vector<unsigned char>{0, 1, 2, 255}));
  EXPECT_EQ(image.mime_type, "image/png");
  EXPECT_EQ(image.time_stamp, "2026-11-08T02:30:00.080Z");
  EXPECT_EQ(std::get<std::string>(image.metadata.at("custom_label")), "door");
  EXPECT_EQ(
    std::get<std::uint64_t>(image.metadata.at("max_counter")),
    std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(std::get<std::int64_t>(image.metadata.at("signed_value")), -3);
  const auto request = calls().at(0);
  EXPECT_EQ(request.at("status"), "completed");
  const auto call = request.at("output").at(0);
  EXPECT_EQ(call.at("name"), "capture_image");
  EXPECT_EQ(
    Json::parse(call.at("arguments").get<std::string>()),
    (Json{{"mode", "latest"}, {"detail", 0}}));
  EXPECT_EQ(call.at("call_id").get<std::string>().find("robot_bt_"), 0);
}

TEST_F(ColorImageTest, AppliesAppendPrependAndBothReplacePositionsToSameVariable)
{
  for (const auto & operation : {"append", "prepend", "replace 0", "replace -1"}) {
    blackboard->set("pictures", samples());
    blackboard->set("operation", std::string(operation));
    auto bt = tree("<GetLatestColorImage operation='{operation}' image_content='{pictures}'/>");
    ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS) << operation;
    const auto pictures = blackboard->get<ImageContent>("pictures");
    const std::string op(operation);
    const bool replacing = op.find("replace") == 0;
    ASSERT_EQ(pictures.images.size(), replacing ? 2 : 3);
    const auto index = op == "prepend" || op == "replace 0" ? 0 : pictures.images.size() - 1;
    EXPECT_EQ(pictures.images.at(index).bytes, (std::vector<unsigned char>{0, 1, 2, 255}));
    EXPECT_EQ(
      pictures.images.at(index == 0 ? 1 : 0).bytes,
      (std::vector<unsigned char>{static_cast<unsigned char>(op == "replace 0" ? 20 : 10)}));
  }
}

TEST_F(ColorImageTest, ClearWritesEmptyCollectionWithoutSubmittingToolCall)
{
  blackboard->set("pictures", samples());
  auto bt = tree("<GetLatestColorImage operation='clear' image_content='{pictures}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard->get<ImageContent>("pictures").images.empty());
  EXPECT_TRUE(calls().empty());
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(calls().empty());
}

TEST_F(ColorImageTest, InvalidOperationsOrIndicesPreserveCollectionWithoutToolCalls)
{
  for (const auto & operation : {"replace", "replace 2", "replace -2", "replace 0 extra",
      "replace 0.5", "replace 99999999999999999999999", "append 1", "clear 1", "bad", ""})
  {
    blackboard->set("pictures", samples());
    blackboard->set("operation", std::string(operation));
    auto bt = tree("<GetLatestColorImage operation='{operation}' image_content='{pictures}'/>");
    EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE) << operation;
    EXPECT_EQ(blackboard->get<ImageContent>("pictures").images.size(), 2);
  }
  blackboard->set("pictures", ImageContent{});
  auto empty = tree("<GetLatestColorImage operation='replace -1' image_content='{pictures}'/>");
  EXPECT_EQ(finish(empty), BT::NodeStatus::FAILURE);
  auto missing = tree("<GetLatestColorImage/>");
  EXPECT_EQ(finish(missing), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(calls().empty());
}

TEST_F(ColorImageTest, ToolFailureOrMalformedImagePreservesPreviousCollection)
{
  blackboard->set("pictures", samples());
  setResult(ToolResult::failure("IMAGE_UNAVAILABLE", "no camera image"));
  auto bt = tree("<GetLatestColorImage operation='replace 0' image_content='{pictures}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard->get<ImageContent>("pictures").images.at(0).bytes,
    (std::vector<unsigned char>{10}));
  auto invalid = ToolResult::success();
  invalid.payload["images"] = Json::array({{{"metadata", Json::object()}}});
  invalid.images = {{{"type", "input_image"}, {"image_url", "data:image/png;base64,!!!!"}}};
  setResult(invalid);
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard->get<ImageContent>("pictures").images.at(0).bytes,
    (std::vector<unsigned char>{10}));
}


TEST_F(ColorImageTest, CallsActualToolManagerAndCaptureImagePluginThroughService)
{
  using GetImage = rgbd_image_queue::srv::GetImage;
  std::atomic_int image_calls{0};
  std::atomic_int image_operation{-1};
  std::atomic_uint image_detail{1};
  auto camera = server->create_service<GetImage>(
    "/color_image_test/image",
    [&](GetImage::Request::SharedPtr request, GetImage::Response::SharedPtr response) {
      image_operation = request->operation;
      image_detail = request->detail;
      ++image_calls;
      response->success = true;
      response->has_color = true;
      response->color.header.frame_id = "camera_optical_frame";
      response->color.header.stamp.sec = 1794096000;
      response->color.format = "png";
      response->color.data = {1, 2, 3, 4};
      response->pair_id = 99;
      response->queue_index = -1;
      response->queue_size = 3;
    });
  const auto directory = std::filesystem::path(VLN_IMAGE_TEST_OUTPUT_DIR);
  std::filesystem::create_directories(directory);
  const auto config = directory / "image_tool_call.json";
  const Json function{{"name", "capture_image"}, {"plugin", "vln_agent/CaptureImage"},
    {"blocking", false}, {"settings", {{"timeout_sec", 1.0}}}};
  const Json group{{"id", "camera"},
    {"definition", std::string(VLN_IMAGE_AGENT_SHARE) + "/plugins/tools/capture_image/tool.json"},
    {"settings", {{"service_name", "/color_image_test/image"}}},
    {"functions", Json::array({function})}};
  const Json configuration{{"tool_manager", {
      {"service_name", "/color_image_test/agent"}, {"max_concurrent_calls", 1},
      {"groups", Json::array({group})}}}};
  std::ofstream(config) << configuration.dump();
  vln_agent::ToolManager manager(server);
  manager.load(config.string());
  auto bt = tree(
    "<GetLatestColorImage service_name='/color_image_test/agent' "
    "image_content='{pictures}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto pictures = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(pictures.images.size(), 1);
  EXPECT_EQ(pictures.images.at(0).bytes, (std::vector<unsigned char>{1, 2, 3, 4}));
  EXPECT_EQ(pictures.images.at(0).mime_type, "image/png");
  EXPECT_FALSE(pictures.images.at(0).time_stamp.empty());
  EXPECT_EQ(
    std::get<std::string>(pictures.images.at(0).metadata.at("frame_id")),
    "camera_optical_frame");
  EXPECT_EQ(std::get<std::uint64_t>(pictures.images.at(0).metadata.at("pair_id")), 99);
  EXPECT_EQ(image_calls.load(), 1);
  EXPECT_EQ(image_operation.load(), GetImage::Request::LATEST_COLOR);
  EXPECT_EQ(image_detail.load(), 0);
  EXPECT_EQ(manager.getStatus().at("completed_count"), 1);
  manager.shutdown();
}

TEST_F(ColorImageTest, SequencePassesImageDirectlyToEventWithFlattenedMetadataAndTfPose)
{
  std::vector<Json> events;
  auto subscription = host->create_subscription<std_msgs::msg::String>(
    "/vln/robot_event", 10, [&](std_msgs::msg::String::ConstSharedPtr message) {
      events.push_back(Json::parse(message->data));
    });
  auto bt = tree(
    "<Sequence><GetLatestColorImage image_content='{pictures}'/>"
    "<SubmitEvent name='ReportImage' image_content='{pictures}'/></Sequence>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (events.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(host);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(events.size(), 1);
  const auto & content = events.at(0).at("content");
  ASSERT_EQ(content.size(), 2);
  EXPECT_EQ(content.at(1).at("image_url"), "data:image/png;base64,AAEC/w==");
  const auto payload = Json::parse(content.at(0).at("text").get<std::string>());
  EXPECT_EQ(payload.at("pose").at("x"), 2.5);
  const auto & metadata = payload.at("images").at(0);
  EXPECT_EQ(metadata.at("custom_label"), "door");
  EXPECT_EQ(metadata.at("time_stamp"), "2026-11-08T02:30:00.080Z");
  EXPECT_EQ(
    metadata.at("max_counter").get<std::uint64_t>(),
    std::numeric_limits<std::uint64_t>::max());
  EXPECT_FALSE(metadata.contains("metadata"));
}
}  // namespace vln_behavior_tree::test
