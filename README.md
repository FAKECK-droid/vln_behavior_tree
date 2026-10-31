# vln_behavior_tree

VLN 导航使用的行为树及 Event 上报节点。`SubmitEvent` 从输入构建事件，向现有
EventManager 的 ROS 话题发布。业务 payload 统一在 `buildPayload()` 中构建。

## 使用

编译并加载工作空间后，在 Nav2 的 `bt_navigator.ros__parameters.plugin_lib_names`
现有列表中追加 `vln_submit_event_bt_node`，即可在行为树中使用：

```xml
<SubmitEvent name="ReportNavigationFailure"
             event="error"
             text="导航执行失败。"/>
```

| 输入端口 | 默认值 | 含义 |
| --- | --- | --- |
| `topic` | `/vln/robot_event` | 事件接收话题，可通过黑板传入 |
| `event` | `status` | 事件类型 |
| `text` | 空字符串 | 事件说明 |
| `image_content` | `[]` | 图片内容 JSON 数组，支持黑板 JSON 值或 JSON 文本 |

每次执行发布一条 `std_msgs/msg/String`。内容沿用 EventManager 接收的原生消息格式：
`role=user`，`content[0]` 为保存业务 JSON 字符串的 `input_text`。
QoS 为 reliable、volatile、KeepLast(10)。发布调用正常完成返回 `SUCCESS`，
输入或发布异常时记录 ROS 日志并返回 `FAILURE`。

图片内容每项使用 `type=input_image`、`image_url` 和可选的 `detail`，可附带
`camera`、`time_stamp` 采样元数据。`buildPayload()` 将元数据按顺序写入
业务 `images`，消息包装时将对应的原生图片内容追加到 `content`。示例输入：

```json
[
  {
    "type": "input_image",
    "image_url": "data:image/jpeg;base64,<已准备好的图片数据>",
    "detail": "auto",
    "camera": "front",
    "time_stamp": "2026-10-31T06:00:00.000Z"
  }
]
```

采样元数据在图片内容项中省略时，对应的业务元数据对象为空。
公共业务字段由 `buildPayload()` 填写：`source=robot`，`origin` 使用节点实例名，
`time_stamp` 使用当前 UTC 毫秒时间。

## 派生节点

包含 `vln_behavior_tree/event_node_base.hpp`，继承 `EventNodeBase`；在
`providedPorts()` 中合并基类端口并增加业务输入，在 `buildPayload()` 中组织业务字段：

```cpp
Json buildPayload() override
{
  auto payload = EventNodeBase::buildPayload();
  payload["data"] = {{"action", getInput<std::string>("action").value()},
    {"state", getInput<std::string>("state").value()}};
  return payload;
}
```

派生插件的 CMake 使用 `find_package(vln_behavior_tree REQUIRED)`，并链接
`vln_behavior_tree::vln_submit_event_bt_node`。按 BehaviorTree.CPP 方式注册自己的节点。

## 开发机检查

在已安装 Humble 的开发机上，从工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vln_behavior_tree --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select vln_behavior_tree --event-handlers console_direct+
colcon test-result --verbose
```

测试使用独立 ROS domain 的本地订阅者，覆盖动态插件加载、消息格式、图片、
黑板输入与派生节点扩展。规划见 `docs/BEHAVIOR_TREE_PLAN.md`。
