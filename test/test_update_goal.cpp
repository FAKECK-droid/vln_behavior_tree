#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "vln_behavior_tree/plugins/actions/update_goal.hpp"

namespace vln_behavior_tree::test
{
using Pose = geometry_msgs::msg::PoseStamped;

/** @brief 模拟规划节点，验证目标输出在同一轮行为树中可被下游读取。 */
class ObserveGoal : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("goal"), BT::OutputPort<Pose>("observed")};
  }

private:
  BT::NodeStatus tick() override
  {
    setOutput("observed", getInput<Pose>("goal").value());
    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateGoalTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("update_goal_test");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    Pose initial;
    initial.header.frame_id = "map";
    initial.header.stamp.sec = 1234;
    initial.header.stamp.nanosec = 5678;
    initial.pose.position.x = 1.0;
    initial.pose.position.y = 2.0;
    initial.pose.position.z = 0.3;
    initial.pose.orientation.w = 1.0;
    blackboard->set("target", initial);
    factory.registerFromPlugin(VLN_UPDATE_GOAL_PLUGIN_PATH);
  }
  BT::Tree tree(const std::string & attributes = "")
  {
    return factory.createTreeFromText(
      "<root><BehaviorTree ID='Main'><UpdateGoal goal='{target}' " +
      attributes + "/></BehaviorTree></root>", blackboard);
  }
  Pose goal() {return blackboard->get<Pose>("target");}
  void setRotation(double roll, double pitch, double yaw)
  {
    auto value = goal();
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    value.pose.orientation.x = q.x();
    value.pose.orientation.y = q.y();
    value.pose.orientation.z = q.z();
    value.pose.orientation.w = q.w();
    blackboard->set("target", value);
  }
  std::array<double, 3> angles(const Pose & value)
  {
    const auto & q = value.pose.orientation;
    std::array<double, 3> result;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(result[0], result[1], result[2]);
    return result;
  }
  rclcpp::Node::SharedPtr node;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
};

TEST_F(UpdateGoalTest, LiteralAndArithmeticExpressionsUpdateSameGoalAndStamp)
{
  auto bt = tree("x='(x + 2) * 3' y='3.5' z='z - 0.1' yaw='pi / 2'");
  const auto before = node->now();
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto after = node->now();
  const auto value = goal();
  EXPECT_DOUBLE_EQ(value.pose.position.x, 9);
  EXPECT_DOUBLE_EQ(value.pose.position.y, 3.5);
  EXPECT_NEAR(value.pose.position.z, 0.2, 1e-12);
  EXPECT_NEAR(angles(value).at(2), std::acos(-1) / 2, 1e-12);
  EXPECT_EQ(value.header.frame_id, "map");
  EXPECT_GE(rclcpp::Time(value.header.stamp).nanoseconds(), before.nanoseconds());
  EXPECT_LE(rclcpp::Time(value.header.stamp).nanoseconds(), after.nanoseconds());
  EXPECT_EQ(
    factory.manifests().at("UpdateGoal").ports.at(
      "goal").direction(), BT::PortDirection::INOUT);
}

TEST_F(UpdateGoalTest, AllFieldsUseOriginalGoalSnapshot)
{
  auto bt = tree("x='y' y='x' yaw='atan2(y, x)'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(goal().pose.position.x, 2);
  EXPECT_DOUBLE_EQ(goal().pose.position.y, 1);
  EXPECT_NEAR(angles(goal()).at(2), std::atan2(2.0, 1.0), 1e-12);
}

TEST_F(UpdateGoalTest, CustomVariablesRefreshWithoutChangingInputMap)
{
  setRotation(0, 0, std::acos(-1) / 2);
  GoalVariables variables{{"step", 0.5}, {"scale", 2}, {"x", 99}, {"pi", 99}};
  blackboard->set("parameters", variables);
  auto bt = tree(
    "variables='{parameters}' x='x + step * cos(yaw)' "
    "y='y + step * sin(yaw)' z='z * scale'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(goal().pose.position.x, 1, 1e-12);
  EXPECT_NEAR(goal().pose.position.y, 2.5, 1e-12);
  EXPECT_NEAR(goal().pose.position.z, 0.6, 1e-12);
  EXPECT_EQ(blackboard->get<GoalVariables>("parameters"), variables);
  variables.at("step") = 1.25;
  variables.at("scale") = 3;
  blackboard->set("parameters", variables);
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(goal().pose.position.y, 3.75, 1e-12);
  EXPECT_NEAR(goal().pose.position.z, 1.8, 1e-12);
}

TEST_F(UpdateGoalTest, SupportsFunctionsParenthesesScientificNotationAndPowers)
{
  const std::vector<std::pair<std::string, double>> cases{
    {"sin(pi / 2) + cos(0)", 2}, {"atan2(1, 0)", std::acos(-1) / 2},
    {"sqrt(abs(-9))", 3}, {"min(5, max(2, 3))", 3},
    {"1e-3 + 2 * (3 - 1)", 4.001}, {"2^3^2", 512}, {"-2^2", -4}};
  auto bt = tree("x='{expression}'");
  for (const auto & item : cases) {
    blackboard->set("expression", item.first);
    ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS) << item.first;
    EXPECT_NEAR(goal().pose.position.x, item.second, 1e-12) << item.first;
  }
}

TEST_F(UpdateGoalTest, UpdatingYawPreservesRollPitchAndProducesUnitQuaternion)
{
  setRotation(0.2, -0.3, 0.4);
  const auto original = goal();
  auto bt = tree("yaw='yaw + pi / 3'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto value = goal();
  const auto rpy = angles(value);
  EXPECT_NEAR(rpy.at(0), 0.2, 1e-12);
  EXPECT_NEAR(rpy.at(1), -0.3, 1e-12);
  EXPECT_NEAR(rpy.at(2), 0.4 + std::acos(-1) / 3, 1e-12);
  const auto & q = value.pose.orientation;
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
  EXPECT_EQ(value.pose.position, original.pose.position);
}

TEST_F(UpdateGoalTest, EmptyOrUnchangedExpressionsPreserveEntireGoalAndTimestamp)
{
  setRotation(0.2, -0.3, 0.4);
  const auto original = goal();
  auto empty = tree();
  ASSERT_EQ(empty.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(goal(), original);
  blackboard->set("blank", std::string(" \t\r\n"));
  auto same = tree("x='x' y='' z='{blank}' yaw='yaw'");
  ASSERT_EQ(same.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(goal(), original);
}

TEST_F(UpdateGoalTest, SyntaxAndNumericErrorsLeaveAllFieldsUntouchedAndCanRecover)
{
  const auto original = goal();
  auto bt = tree("x='x + 1' yaw='{expression}'");
  for (const auto & expression : {"yaw +", "unknown + 1", "atan2(1)", "min(1)",
      "(1 + 2", "1 / 0", "sqrt(-1)", "1e308 * 1e308"})
  {
    blackboard->set("expression", std::string(expression));
    EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE) << expression;
    EXPECT_EQ(goal(), original) << expression;
  }
  blackboard->set("expression", std::string("0.5"));
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(goal().pose.position.x, 2);
  EXPECT_NEAR(angles(goal()).at(2), 0.5, 1e-12);
}

TEST_F(UpdateGoalTest, MissingInputsAndInvalidPoseReturnFailureWithoutWritingGoal)
{
  const auto original = goal();
  auto missing_expression = tree("x='{missing_expression}'");
  EXPECT_EQ(missing_expression.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(goal(), original);
  auto missing_variables = tree("x='1' variables='{missing_variables}'");
  EXPECT_EQ(missing_variables.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(goal(), original);
  auto invalid = original;
  invalid.pose.orientation.w = 0;
  blackboard->set("target", invalid);
  auto bt = tree("x='x + 1'");
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(goal(), invalid);
  invalid = original;
  invalid.pose.position.y = std::numeric_limits<double>::infinity();
  blackboard->set("target", invalid);
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_DOUBLE_EQ(goal().pose.position.x, original.pose.position.x);
  auto no_goal = factory.createTreeFromText(
    "<root><BehaviorTree ID='Main'><UpdateGoal goal='{missing_goal}' x='1'/></BehaviorTree></root>",
    blackboard);
  EXPECT_EQ(no_goal.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(UpdateGoalTest, RelativeOperationsAccumulateAndExpressionInputsRefresh)
{
  blackboard->set("expression", std::string("x + 1"));
  auto bt = tree("x='{expression}'");
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_DOUBLE_EQ(goal().pose.position.x, 4);
  blackboard->set("expression", std::string("x * 2"));
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(goal().pose.position.x, 8);
}

TEST_F(UpdateGoalTest, ChangedGoalUsesRosClockIncludingSimulation)
{
  node = std::make_shared<rclcpp::Node>(
    "update_goal_sim_clock", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}));
  blackboard->set("node", node);
  ASSERT_TRUE(node->get_clock()->ros_time_is_active());
  auto bt = tree("x='x + 1'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  const builtin_interfaces::msg::Time expected = node->now();
  EXPECT_EQ(goal().header.stamp, expected);
}

TEST_F(UpdateGoalTest, DownstreamNodeReadsUpdatedTypedGoalInSameTick)
{
  factory.registerNodeType<ObserveGoal>("ObserveGoal");
  auto bt = factory.createTreeFromText(
    "<root><BehaviorTree ID='Main'><Sequence>"
    "<UpdateGoal goal='{target}' x='x + 2' yaw='pi / 2'/>"
    "<ObserveGoal goal='{target}' observed='{seen}'/>"
    "</Sequence></BehaviorTree></root>", blackboard);
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<Pose>("seen"), goal());
  EXPECT_DOUBLE_EQ(blackboard->get<Pose>("seen").pose.position.x, 3);
}
}  // namespace vln_behavior_tree::test
