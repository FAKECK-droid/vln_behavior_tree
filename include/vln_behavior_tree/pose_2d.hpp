#ifndef VLN_BEHAVIOR_TREE__POSE_2D_HPP_
#define VLN_BEHAVIOR_TREE__POSE_2D_HPP_

#include <string>

namespace vln_behavior_tree
{
/** @brief 通过黑板传递的二维小车位姿；距离使用米，yaw 为逆时针弧度。 */
struct Pose2D
{
  std::string frame_id;          ///< 位姿所在坐标系，如 map 或 odom。
  double x{0.0};                ///< x 坐标，单位米。
  double y{0.0};                ///< y 坐标，单位米。
  double yaw{0.0};              ///< 相对坐标系正 x 轴的逆时针弧度角。
  std::string time_stamp{};      ///< 可选的位姿采样时间，UTC 三位毫秒字符串。
};
}  // namespace vln_behavior_tree

#endif  // VLN_BEHAVIOR_TREE__POSE_2D_HPP_
