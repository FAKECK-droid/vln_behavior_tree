#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace vln_behavior_tree
{
namespace
{
using Pose = geometry_msgs::msg::PoseStamped;
using Goals = std::vector<Pose>;
using Path = nav_msgs::msg::Path;

bool withinPositionTolerance(const Pose & a, const Pose & b, double tolerance)
{
  const auto & ap = a.pose.position;
  const auto & bp = b.pose.position;
  return a.header.frame_id == b.header.frame_id && std::isfinite(tolerance) &&
         std::hypot(ap.x - bp.x, ap.y - bp.y) <= tolerance;
}

class FollowWaypoints : public BT::DecoratorNode
{
public:
  FollowWaypoints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {
    node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
    node_->get_parameter_or("transform_tolerance", transform_timeout_, 0.1);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::BidirectionalPort<Goals>("goals"),
      BT::OutputPort<Pose>("waypoint"), BT::OutputPort<bool>("is_final"),
      BT::OutputPort<Path>("path"), BT::OutputPort<std::uint64_t>("route_revision")};
  }

  void halt() override
  {
    remaining_.reset();
    BT::DecoratorNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    const auto input = getInput<Goals>("goals");
    if (!input || input.value().empty()) {
      resetChild();
      return BT::NodeStatus::FAILURE;
    }

    // Humble replaces {goals} on accepted Action preemption. Halt the old
    // child before any new planner or controller goal can be sent.
    if (!remaining_ || *remaining_ != input.value()) {
      resetChild();
      setOutput("path", Path{});
      auto goals = input.value();
      if (!transformGoals(goals)) {
        return BT::NodeStatus::FAILURE;
      }
      remaining_ = std::move(goals);
      ++route_revision_;
      setOutput("goals", *remaining_);
    }

    setStatus(BT::NodeStatus::RUNNING);
    setOutput("waypoint", remaining_->front());
    setOutput("is_final", remaining_->size() == 1);
    if (config().output_ports.count("route_revision")) {
      setOutput("route_revision", route_revision_);
    }
    const auto result = child_node_->executeTick();
    if (result != BT::NodeStatus::SUCCESS) {
      return result;
    }

    resetChild();
    remaining_->erase(remaining_->begin());
    setOutput("goals", *remaining_);
    if (remaining_->empty()) {
      return BT::NodeStatus::SUCCESS;
    }
    // Let Nav2 process cancel/preempt before starting work on the next point.
    setOutput("path", Path{});
    return BT::NodeStatus::RUNNING;
  }

  bool transformGoals(Goals & goals) const
  {
    // Resolve goals in moving frames once using their original timestamps.
    // A later waypoint may start long after those transforms expire.
    for (auto & goal : goals) {
      if (goal.header.frame_id != global_frame_) {
        Pose fixed_goal;
        if (!nav2_util::transformPoseInTargetFrame(
            goal, fixed_goal, *tf_, global_frame_, transform_timeout_))
        {
          RCLCPP_ERROR(
            node_->get_logger(), "Cannot transform VLN waypoint to %s",
            global_frame_.c_str());
          return false;
        }
        goal = std::move(fixed_goal);
      }
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_;
  double transform_timeout_;
  std::optional<Goals> remaining_;
  std::uint64_t route_revision_{0};
};

/** @brief 并行跟踪路径与接近事件，事件失败独立于导航结果。 */
class ReportNearWaypoint : public BT::ControlNode
{
public:
  ReportNearWaypoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {
    const auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    node->get_parameter_or("robot_base_frame", robot_frame_, std::string("base_link"));
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("waypoint"), BT::InputPort<bool>("is_final"),
      BT::InputPort<std::uint64_t>("route_revision"),
      BT::InputPort<double>("distance_m", 1.0, "触发时与末航点的最大平面距离，单位米")};
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {throw BT::LogicError("ReportNearWaypoint 需要路径跟踪和上报两个子节点");}
    auto & follow = *children_nodes_[0];
    auto & report = *children_nodes_[1];
    const auto follow_status = follow.status() == BT::NodeStatus::SUCCESS ?
      BT::NodeStatus::SUCCESS : follow.executeTick();
    if (follow_status == BT::NodeStatus::FAILURE) {
      resetChildren();
      return follow_status;
    }

    auto report_status = report.status();
    if (report_status == BT::NodeStatus::RUNNING) {
      report_status = report.executeTick();
    } else if (getInput<bool>("is_final").value()) {
      const auto revision = getInput<std::uint64_t>("route_revision").value();
      if (revision != reported_revision_ && nearWaypoint()) {
        reported_revision_ = revision;
        report_status = report.executeTick();
      }
    }
    if (follow_status == BT::NodeStatus::RUNNING || report_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    resetChildren();
    return follow_status;
  }

  bool nearWaypoint() const
  {
    const auto waypoint = getInput<Pose>("waypoint").value();
    const auto distance = getInput<double>("distance_m").value();
    Pose robot;
    return tf_->canTransform(waypoint.header.frame_id, robot_frame_, tf2::TimePointZero) &&
           nav2_util::getCurrentPose(robot, *tf_, waypoint.header.frame_id, robot_frame_, 0.0) &&
           withinPositionTolerance(robot, waypoint, distance);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_frame_;
  std::uint64_t reported_revision_{0};
};

class ValidateWaypointPath : public BT::SyncActionNode
{
public:
  ValidateWaypointPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("waypoint"), BT::InputPort<Path>("candidate_path"),
      BT::InputPort<double>("xy_tolerance"), BT::OutputPort<Path>("path")};
  }

private:
  BT::NodeStatus tick() override
  {
    const auto waypoint = getInput<Pose>("waypoint");
    const auto candidate = getInput<Path>("candidate_path");
    const auto tolerance = getInput<double>("xy_tolerance");
    if (!waypoint || !candidate || candidate.value().poses.empty() || !tolerance) {
      return BT::NodeStatus::FAILURE;
    }

    auto endpoint = candidate.value().poses.back();
    endpoint.header.frame_id = candidate.value().header.frame_id;
    if (!withinPositionTolerance(endpoint, waypoint.value(), tolerance.value())) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Rejected VLN path endpoint (%.2f, %.2f) in %s for waypoint (%.2f, %.2f) in %s",
        endpoint.pose.position.x, endpoint.pose.position.y, endpoint.header.frame_id.c_str(),
        waypoint.value().pose.position.x, waypoint.value().pose.position.y,
        waypoint.value().header.frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return setOutput("path", candidate.value()) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  rclcpp::Node::SharedPtr node_;
};

class WaypointPositionReached : public BT::ConditionNode
{
public:
  WaypointPositionReached(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {
    const auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    node->get_parameter_or("robot_base_frame", robot_frame_, std::string("base_link"));
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("waypoint"), BT::InputPort<double>("xy_tolerance"),
      BT::InputPort<bool>("is_final", false, "True for the last waypoint of the request")};
  }

private:
  BT::NodeStatus tick() override
  {
    const auto is_final = getInput<bool>("is_final");
    const auto waypoint = getInput<Pose>("waypoint");
    const auto tolerance = getInput<double>("xy_tolerance");
    if (!is_final || is_final.value() || !waypoint || !tolerance) {
      return BT::NodeStatus::FAILURE;
    }

    const auto & target = waypoint.value();
    Pose robot;
    if (!tf_->canTransform(target.header.frame_id, robot_frame_, tf2::TimePointZero) ||
      !nav2_util::getCurrentPose(robot, *tf_, target.header.frame_id, robot_frame_, 0.0))
    {
      return BT::NodeStatus::FAILURE;
    }
    return withinPositionTolerance(robot, target, tolerance.value()) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_frame_;
};
}  // namespace

void registerWaypointNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<FollowWaypoints>("FollowWaypoints");
  factory.registerNodeType<ReportNearWaypoint>("ReportNearWaypoint");
  factory.registerNodeType<ValidateWaypointPath>("ValidateWaypointPath");
  factory.registerNodeType<WaypointPositionReached>("WaypointPositionReached");
}
}  // namespace vln_behavior_tree
