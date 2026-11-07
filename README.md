# vln_behavior_tree

VLN 导航行为树及 Event 上报节点。`SubmitEvent` 从输入构建事件并发布给
EventManager，业务 JSON、图片元数据和 Base64 编码统一在 `buildPayload()` 中完成。

## 使用

编译并加载工作空间，在 Nav2 的 `bt_navigator.ros__parameters.plugin_lib_names`
现有列表中追加 `vln_submit_event_bt_node`：

```xml
<SubmitEvent name="ReportNavigationFailure"
             event="error"
             message="导航执行失败。"/>
```

| 输入端口 | 默认值 | 含义 |
| --- | --- | --- |
| `topic` | `/vln/robot_event` | 事件接收话题，可通过黑板传入 |
| `event` | `status` | 顶层事件类型 |
| `message` | 空字符串 | 事件说明，写入 `data.message` |
| `time_stamp` | 当前 UTC 时间 | 可选的事件发生时间，UTC 三位毫秒字符串 |
| `pose` | 省略 | 黑板中的 `Pose2D`，表示对应的小车位姿 |
| `image_content` | 省略 | 黑板中的 `ImageContent`，包含图片字节、采样信息和自定义元数据 |

每次执行发布一条 `std_msgs/msg/String`。内容为 `role=user` 原生消息，
`content[0]` 是保存业务 JSON 字符串的 `input_text`，图片依次放在后续 `input_image`。
QoS 为 reliable、volatile、KeepLast(10)。发布调用完成返回 `SUCCESS`，输入或发布异常
记录 ROS 日志并返回 `FAILURE`。

上述例子的业务 JSON 为：

```json
{
  "source": "robot",
  "time_stamp": "2026-11-07T02:30:00.125Z",
  "origin": "ReportNavigationFailure",
  "event": "error",
  "data": {"message": "导航执行失败。"}
}
```

`origin` 使用节点实例名，`origin/event` 与 `data` 同级。`data` 默认空对象；
具体业务载荷由 `buildPayload()` 填写。公共字段复用 `vln_agent::utils::businessPayload()`。

## 位姿和采样时间

`pose_2d.hpp` 提供纯 C++ 的 `Pose2D`：frame_id、米制 x/y、逆时针弧度 yaw，
以及可选的 UTC 毫秒采样时间 time_stamp。事件时间与位姿采样时间分别传入：

```cpp
#include <vln_behavior_tree/pose_2d.hpp>

blackboard->set("event_time", std::string("2026-11-07T02:30:00.125Z"));
blackboard->set("robot_pose", vln_behavior_tree::Pose2D{
  "map", 1.2, 2.3, 0.5, "2026-11-07T02:30:00.080Z"});
```

```xml
<SubmitEvent name="ReportNavigationCompleted"
             event="navigation_completed"
             message="已到达目标位置。"
             time_stamp="{event_time}"
             pose="{robot_pose}"/>
```

基类在 `buildPayload()` 内通过共用的 `posePayload()` 构造顶层 `pose`；
采样时间填写在 `pose.time_stamp`，传入的事件时间保持原值。

## 图片输入

`image_content.hpp` 提供 `ImageContent`，其 `images` 数组的每项包含：

- `bytes`：JPEG、PNG 等图片文件字节，`std::vector<unsigned char>`。
- `metadata`：自定义 `std::map<std::string, MetadataValue>`；值支持字符串、布尔、int64 和 double。
- `mime_type`：默认 `image/jpeg`，PNG 使用 `image/png`。
- `time_stamp`：可选的图片采样时间，空字符串表示省略。
- `pose`：可选的采图时小车位姿，类型为 `std::optional<Pose2D>`。

提供方填写 C++ 数据并写入黑板：

```cpp
#include <vln_behavior_tree/image_content.hpp>

vln_behavior_tree::ImageContent pictures;
pictures.images.push_back({
  jpeg_bytes,
  {{"label", std::string("door")}, {"score", 0.97},
    {"sequence", std::int64_t{7}}, {"usable", true}},
  "image/jpeg"});
pictures.images.back().time_stamp = "2026-11-07T02:30:00.080Z";
blackboard->set("pictures", pictures);
```

```xml
<SubmitEvent name="ReportObservation"
             message="请结合图片判断当前情况。"
             image_content="{pictures}"/>
```

`buildPayload()` 读取一次图片集合，为每张图片构建一个业务 `images` 描述和一个
原生 `input_image`。描述与图片按数组顺序逐张对应：

```json
{
  "time_stamp": "2026-11-07T02:30:00.080Z",
  "metadata": {"label": "door", "score": 0.97, "sequence": 7, "usable": true}
}
```

可选的图片位姿放在描述的 `pose` 中。自定义 metadata 保留提供方的全部键值，
与公共采样字段分开；空元数据对应 `{"metadata":{}}`。省略图片端口或传入空集合时
发送纯文字事件。发布前复用 Agent 的图片数量检查，确保 `images` 与原生图片逐张对应。

## 派生节点

继承 `EventNodeBase`，在 `providedPorts()` 中合并基类端口并增加业务输入。
`buildPayload()` 返回的 `EventPayload.business` 保存业务 JSON，
`image_content` 保存已编码的原生图片数组：

```cpp
EventPayload buildPayload() override
{
  auto event = EventNodeBase::buildPayload();
  auto & data = event.business["data"];
  data["action"] = getInput<std::string>("action").value();
  data["state"] = getInput<std::string>("state").value();
  return event;
}
```

派生插件使用 `find_package(vln_behavior_tree REQUIRED)` 并链接
`vln_behavior_tree::vln_submit_event_bt_node`，按 BehaviorTree.CPP 方式注册。

## 编译与检查

使用具有统一业务封装接口的当前 `vln_agent` 版本，在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vln_behavior_tree --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select vln_behavior_tree --event-handlers console_direct+
colcon test-result --verbose
```

测试使用独立 ROS domain 的本地订阅者，覆盖动态插件加载、统一业务格式、
采样时间、位姿、自定义图片元数据、派生节点扩展及 Agent 输入检查。
规划见 `docs/BEHAVIOR_TREE_PLAN.md`。
