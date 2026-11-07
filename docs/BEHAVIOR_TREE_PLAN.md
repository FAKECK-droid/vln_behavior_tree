# VLN Event 上报节点规划

更新日期：2026-11-07。通用事件基类适配当前 Agent 业务 JSON 规范。

## 1. 职责

Event 节点从输入构建机器事件，发布给 EventManager，供模型决策参考。
行为树决定上报时机和后续动作；观测和动作状态由其他节点准备，通过黑板或端口传入。

```text
其他节点准备数据 → 黑板 / 输入端口 → Event 节点 → /vln/robot_event
```

## 2. 构成和执行流程

`EventNodeBase` 继承 `BT::SyncActionNode`，可注册为通用 `SubmitEvent`，也可派生扩展。

1. `buildPayload()` 读取输入，构建公共字段和 data，转换图片元数据并编码 Base64。
2. 检查图片与 images 元数据数量，将业务 JSON 和图片包装为一条原生消息。
3. 发布后返回 SUCCESS；输入、构建或发布异常时记录日志并返回 FAILURE。

上报频率、重试和后续行为由树中其他节点组合决定。

## 3. 输入和业务格式

| 端口 | 类型 | 用途 |
| --- | --- | --- |
| topic | string | 默认 /vln/robot_event |
| event | string | 顶层事件类型，默认 status |
| message | string | 可选说明，写入 data.message |
| time_stamp | string，可选 | 事件发生时刻，省略时使用当前 UTC 毫秒时间 |
| pose | Pose2D，可选 | 对应的小车位姿及可选采样时间 |
| image_content | ImageContent，可选 | 图片字节、采样信息和自定义 metadata |

```json
{
  "source": "robot",
  "time_stamp": "2026-11-07T02:30:00.125Z",
  "origin": "ReportNavigationFailure",
  "event": "error",
  "pose": {
    "frame_id": "map",
    "time_stamp": "2026-11-07T02:30:00.080Z",
    "x": 1.2,
    "y": 2.3,
    "yaw": 0.5
  },
  "data": {
    "message": "路径跟随失败。",
    "action": "FollowPath",
    "state": "FAILURE",
    "distance_remaining_m": 2.4
  }
}
```

source/time_stamp/data 由共用 businessPayload() 构建，origin 为节点实例名，
event 与 origin 是机器事件的顶层字段。具体动作、错误和观测载荷在 data 中构建。
位姿由纯 C++ Pose2D 输入，通过共用 posePayload() 序列化，x/y 单位米，yaw 单位弧度。

`ImageContent::Image` 保存 JPEG/PNG 文件字节、mime_type、自定义 metadata map，
并支持可选的 time_stamp 和 pose。图片元数据形式为：

```json
{
  "time_stamp": "2026-11-07T02:30:00.080Z",
  "metadata": {"label": "door", "score": 0.97}
}
```

每张图片按顺序对应一个 images 元素和一个原生 input_image。
metadata 内容完全由提供方定义，公共采样字段与自定义 metadata 分开放置。
提供方使用普通 C++ 类型；所有 Event JSON 构建与图片编码集中在 buildPayload()。

## 4. 接口和派生

```cpp
class EventNodeBase : public BT::SyncActionNode
{
public:
  EventNodeBase(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  virtual EventPayload buildPayload();

private:
  BT::NodeStatus tick() override;
};
```

EventPayload 包含 business 和已编码的 image_content。派生类调用基类构建公共字段，
再向 business["data"] 添加自身业务字段；这样保留基类生成的 data.message。

## 5. 发布与使用

话题 /vln/robot_event，类型 std_msgs/msg/String，QoS 为 reliable、volatile、KeepLast(10)。
消息格式为 role=user，content 第一项为业务 JSON 字符串 input_text，后面依次放 input_image。
tick() 整体封装已构建的数据，复用 Agent 的图片对应关系检查。

```xml
<SubmitEvent name="ReportNavigationFailure"
             event="error"
             message="导航执行失败。"/>
```

可将该节点放在成功、恢复、失败或其他上报分支。具体接入两棵 Nav2 导航树的位置另行确定：

- behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
- behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml

## 6. 文件与验证

```text
include/vln_behavior_tree/pose_2d.hpp
include/vln_behavior_tree/image_content.hpp
include/vln_behavior_tree/event_node_base.hpp
src/event_node_base.cpp
```

验证覆盖公共字段、data 载荷、事件与采样时间、位姿、自定义元数据、图片顺序，
以及派生节点与接收端公共输入检查的兼容性。测试使用普通 ROS 本地订阅者。

2026-10-31：实现通用 EventNodeBase 与 ImageContent，集中构建业务数据和编码图片。
2026-11-07：适配统一业务字段，message 写入 data，图片描述使用 time_stamp/pose/metadata。
