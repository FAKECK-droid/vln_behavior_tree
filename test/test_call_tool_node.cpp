#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <vln_agent/tools/tool_manager.hpp>

#include "vln_behavior_tree/call_tool_node_base.hpp"

namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
using Json = vln_agent::Json;
using ToolResult = vln_agent::ToolResult;
using Service = vln_agent::srv::ToolCall;

/** @brief 派生节点只读取普通类型端口，参数 JSON 由 buildArguments 组织。 */
class SurroundingsTool : public CallToolNodeBase
{
public:
  using CallToolNodeBase::CallToolNodeBase;
  static BT::PortsList providedPorts()
  {
    auto ports = CallToolNodeBase::providedPorts();
    ports.at("tool") = BT::InputPort<std::string>("tool", "get_surroundings", "工具名").second;
    ports.insert(BT::InputPort<std::string>("costmap", "local", "代价地图来源"));
    ports.insert(BT::InputPort<double>("range_m", 10.0, "观察范围"));
    return ports;
  }

protected:
  Json buildArguments() override
  {
    return {{"costmap", getInput<std::string>("costmap").value()},
      {"range_m", getInput<double>("range_m").value()}};
  }
};

/** @brief 派生节点可以按业务需要处理失败结果，原始结果仍写入黑板。 */
class AcceptToolFailure : public CallToolNodeBase
{
public:
  using CallToolNodeBase::CallToolNodeBase;

protected:
  BT::NodeStatus onResult(const ToolResult &) override {return BT::NodeStatus::SUCCESS;}
};

class CallToolNodeTest : public ::testing::Test
{
protected:
  using Reply = std::function<void (const Json &, Service::Response &)>;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    host = std::make_shared<rclcpp::Node>("call_tool_bt_test");
    server = std::make_shared<rclcpp::Node>("call_tool_bt_service");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", host);
    blackboard->set("bt_loop_duration", 10ms);
    blackboard->set("server_timeout", 20ms);
    // Nav2 读取的等待配置类型是 milliseconds。
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(2000));
    responder = [](const Json & request, Service::Response & reply) {
        const auto & call = request.at("output").at(0);
        auto result = ToolResult::success(Json::parse(call.at("arguments").get<std::string>()));
        reply.outputs_json = Json::array(
          {vln_agent::utils::functionCallOutput(
              call.at("call_id").get<std::string>(), result.payload, result.images)}).dump();
      };
    createService("/vln/tool_call");
    executor.add_node(server);
    spinner = std::thread([this] {executor.spin();});
    factory.registerFromPlugin(VLN_CALL_TOOL_PLUGIN_PATH);
  }

  void TearDown() override
  {
    executor.cancel();
    spinner.join();
  }

  void createService(const std::string & name)
  {
    service = server->create_service<Service>(
      name,
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr reply) {
        const auto parsed = Json::parse(request->request_json);
        Reply handler;
        {
          std::lock_guard<std::mutex> lock(mutex);
          requests.push_back(parsed);
          handler = responder;
        }
        handler(parsed, *reply);
        ++completed;
      });
  }

  void respond(Reply handler)
  {
    std::lock_guard<std::mutex> lock(mutex);
    responder = std::move(handler);
  }

  std::vector<Json> calls()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return requests;
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

  bool waitCompleted(int count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (completed.load() >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr host, server;
  rclcpp::Service<Service>::SharedPtr service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  std::mutex mutex;
  std::vector<Json> requests;
  Reply responder;
  std::atomic_int completed{0};
};

TEST_F(CallToolNodeTest, DefaultServiceWaitsForToolAndReturnsFreshInputsWithUniqueIds)
{
  respond(
    [](const Json & request, Service::Response & reply) {
      std::this_thread::sleep_for(60ms);
      const auto & call = request.at("output").at(0);
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            call.at("call_id").get<std::string>(),
            ToolResult::success(
              Json::parse(
                call.at(
                  "arguments").get<std::string>())).payload, {})}).dump();
    });
  blackboard->set("arguments", std::string("{\"range_m\":10}"));
  blackboard->set("tool_result", ToolResult::success({{"old", true}}));
  auto bt = tree(
    "<CallTool tool='get_surroundings' arguments='{arguments}' result='{tool_result}'/>");
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(blackboard->get<ToolResult>("tool_result").payload.is_null());
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<ToolResult>("tool_result").payload.at("data"), (Json{{"range_m", 10}}));
  auto first = calls().at(0);
  EXPECT_EQ(first.at("status"), "completed");
  EXPECT_EQ(first.at("output").at(0).at("type"), "function_call");
  EXPECT_EQ(first.at("output").at(0).at("name"), "get_surroundings");
  EXPECT_EQ(first.at("output").at(0).at("call_id").get<std::string>().find("robot_bt_"), 0u);
  blackboard->set("arguments", std::string("{\"range_m\":20}"));
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<ToolResult>("tool_result").payload.at("data"), (Json{{"range_m", 20}}));
  const auto second = calls().at(1);
  EXPECT_NE(first.at("output").at(0).at("call_id"), second.at("output").at(0).at("call_id"));
}

TEST_F(CallToolNodeTest, DerivedNodeBuildsArgumentsFromTypedPorts)
{
  factory.registerNodeType<SurroundingsTool>("ReadSurroundings");
  blackboard->set("range", 4.5);
  auto bt = tree("<ReadSurroundings costmap='global' range_m='{range}' result='{result}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    blackboard->get<ToolResult>("result").payload.at("data"),
    (Json{{"costmap", "global"}, {"range_m", 4.5}}));
}

TEST_F(CallToolNodeTest, KeepsBusinessPayloadMetadataAndNativeImagesUnchanged)
{
  auto expected = ToolResult::success({{"environment_id", "env_fixture"}});
  expected.payload["time_stamp"] = "2026-11-07T02:30:00.125Z";
  expected.payload["pose"] = vln_agent::utils::posePayload("map", 1.2, 2.3, 0.5);
  expected.payload["images"] = Json::array(
    {
      {{"time_stamp", "2026-11-07T02:30:00.080Z"}, {"metadata", {{"label", "door"}, {"score",
              0.9}}}},
      {{"metadata", {{"custom_flag", true}}}}});
  expected.images = {{{"type", "input_image"},
    {"image_url", "data:image/png;base64," + std::string(128 * 1024, 'A')}, {"detail", "high"}},
    {{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}, {"detail", "auto"}}};
  respond(
    [expected](const Json & request, Service::Response & reply) {
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            expected.payload, expected.images)}).dump();
    });
  auto bt = tree("<CallTool tool='capture_image' result='{result}'/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto result = blackboard->get<ToolResult>("result");
  EXPECT_EQ(result.payload, expected.payload);
  EXPECT_EQ(result.images, expected.images);
}

TEST_F(CallToolNodeTest, ToolFailureRemainsReadableAndReturnsFailure)
{
  const auto expected = ToolResult::failure("COSTMAP_UNAVAILABLE", "代价地图不可用");
  respond(
    [expected](const Json & request, Service::Response & reply) {
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at(
              "call_id").get<std::string>(), expected.payload, {})}).dump();
    });
  auto bt = tree("<CallTool tool='get_surroundings' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard->get<ToolResult>("result").payload, expected.payload);
}

TEST_F(CallToolNodeTest, PhysicalNavigationStatusDoesNotReplaceToolOk)
{
  respond(
    [](const Json & request, Service::Response & reply) {
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            ToolResult::success({{"status", "failed"}}).payload, {})}).dump();
    });
  auto bt = tree("<CallTool tool='navigate_to_pose_get_feedback' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<ToolResult>("result").payload.at("data").at("status"), "failed");
}

TEST_F(CallToolNodeTest, ServiceErrorClearsOldResultAndReturnsFailure)
{
  respond(
    [](const Json &, Service::Response & reply) {
      reply.error = "请求未提交";
      reply.outputs_json = "[]";
    });
  blackboard->set("result", ToolResult::success({{"old", true}}));
  auto bt = tree("<CallTool tool='example' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.is_null());
}

TEST_F(CallToolNodeTest, RejectsMalformedOrUncorrelatedResponses)
{
  for (const auto & invalid : {std::string("{"), std::string("[]"),
      std::string("[{\"type\":\"function_call_output\",\"call_id\":\"other\",\"output\":[]}]"),
      std::string("{}")})
  {
    respond([invalid](const Json &, Service::Response & reply) {reply.outputs_json = invalid;});
    auto bt = tree("<CallTool tool='example' result='{result}'/>");
    EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.is_null());
  }
}

TEST_F(CallToolNodeTest, InvalidArgumentsAndMissingToolDoNotSubmit)
{
  for (const auto & body : {"<CallTool tool='example' arguments='[' result='{result}'/>",
      "<CallTool tool='example' arguments='[]' result='{result}'/>",
      "<CallTool result='{result}'/>"})
  {
    auto bt = tree(body);
    EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.is_null());
  }
  EXPECT_TRUE(calls().empty());
}

TEST_F(CallToolNodeTest, TimeoutDropsLocalWaitAndNextExecutionCanSucceed)
{
  respond(
    [](const Json & request, Service::Response & reply) {
      std::this_thread::sleep_for(100ms);
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            ToolResult::success().payload, {})}).dump();
    });
  auto bt = tree("<CallTool tool='example' server_timeout='30' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.is_null());
  ASSERT_TRUE(waitCompleted(1));
  respond(
    [](const Json & request, Service::Response & reply) {
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            ToolResult::success({{"fresh", true}}).payload, {})}).dump();
    });
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.at("data").at("fresh"));
  EXPECT_EQ(calls().size(), 2u);
}

TEST_F(CallToolNodeTest, HaltStopsWaitingWithoutStoppingTheSubmittedServiceWork)
{
  respond(
    [](const Json & request, Service::Response & reply) {
      std::this_thread::sleep_for(60ms);
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            ToolResult::success().payload, {})}).dump();
    });
  auto bt = tree("<CallTool tool='example' result='{result}'/>");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::RUNNING);
  bt.haltTree();
  EXPECT_EQ(bt.rootNode()->status(), BT::NodeStatus::IDLE);
  ASSERT_TRUE(waitCompleted(1));
  EXPECT_TRUE(blackboard->get<ToolResult>("result").payload.is_null());
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  const auto requests = calls();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_NE(
    requests.at(0).at("output").at(0).at("call_id"),
    requests.at(1).at("output").at(0).at("call_id"));
}

TEST_F(CallToolNodeTest, DerivedResultHookCanApplyItsOwnSuccessPolicy)
{
  factory.registerNodeType<AcceptToolFailure>("AcceptFailure");
  respond(
    [](const Json & request, Service::Response & reply) {
      reply.outputs_json = Json::array(
        {vln_agent::utils::functionCallOutput(
            request.at("output").at(0).at("call_id").get<std::string>(),
            ToolResult::failure("UNAVAILABLE", "服务未就绪").payload, {})}).dump();
    });
  auto bt = tree("<AcceptFailure tool='example' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(blackboard->get<ToolResult>("result").payload.at("ok"));
}

TEST_F(CallToolNodeTest, CallsTheActualAgentServiceAndReceivesItsCorrelatedToolError)
{
  const auto directory = std::filesystem::path(VLN_CALL_TEST_OUTPUT_DIR);
  std::filesystem::create_directories(directory);
  const auto config = directory / "tool_call_agent.json";
  std::ofstream(config) << Json{{"tool_manager", {
      {"service_name", "/call_tool_node_test/agent"}, {"max_concurrent_calls", 1},
      {"groups", Json::array()}}}}.dump();
  vln_agent::ToolManager manager(server);
  manager.load(config.string());
  auto bt = tree(
    "<CallTool tool='not_enabled' service_name='/call_tool_node_test/agent' result='{result}'/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  const auto result = blackboard->get<ToolResult>("result");
  EXPECT_EQ(result.payload.at("code"), "UNKNOWN_TOOL");
  EXPECT_EQ(result.payload.at("source"), "robot");
  EXPECT_EQ(result.payload.at("data"), Json::object());
  EXPECT_EQ(manager.getStatus().at("completed_count"), 1);
  manager.shutdown();
}
}  // namespace vln_behavior_tree::test
