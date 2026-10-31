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
| `image_content` | 空集合 | 黑板中的 `ImageContent` 对象，包含图片字节与自定义元数据 |

每次执行发布一条 `std_msgs/msg/String`。内容沿用 EventManager 接收的原生消息格式：
`role=user`，`content[0]` 为保存业务 JSON 字符串的 `input_text`。
QoS 为 reliable、volatile、KeepLast(10)。发布调用正常完成返回 `SUCCESS`，
输入或发布异常时记录 ROS 日志并返回 `FAILURE`。

## 图片输入

`image_content.hpp` 提供纯 C++ 的 `ImageContent` 容器。它的 `images` 为数组，每项包含：

- `bytes`：JPEG、PNG 等图片文件字节，类型为 `std::vector<unsigned char>`。
- `metadata`：`std::map<std::string, MetadataValue>`，键名由提供方自定义；
  值支持 `std::string`、`bool`、`std::int64_t`、`double`。
- `mime_type`：图片媒体类型，默认 `image/jpeg`，PNG 使用 `image/png`。

上游将图片字节和普通 C++ 元数据写入黑板：

```cpp
#include <vln_behavior_tree/image_content.hpp>

vln_behavior_tree::ImageContent pictures;
pictures.images.push_back({
  jpeg_bytes,
  {{"label", std::string("door")}, {"score", 0.97},
    {"sequence", std::int64_t{7}}, {"usable", true}},
  "image/jpeg"});
blackboard->set("pictures", pictures);
```

```xml
<SubmitEvent name="ReportObservation"
             text="请结合图片判断当前情况。"
             image_content="{pictures}"/>
```

`buildPayload()` 读取一次 `ImageContent`，将每项的整个元数据 map 转换为业务
`images` 中的一个对象，并将图片字节编码为 Base64 Data URL。元数据与图片按数组顺序对应。
空元数据 map 对应 `{}`；省略图片端口或传入空集合时发送纯文字事件。

时间和 Base64 编码复用 `vln_agent_utils` 的 `utcNow()`、`encodeBase64()`。
公共业务字段为 `source=robot`、节点实例名 `origin` 和 UTC 毫秒 `time_stamp`。

## 派生节点

包含 `vln_behavior_tree/event_node_base.hpp`，继承 `EventNodeBase`；在
`providedPorts()` 中合并基类端口并增加业务输入。`buildPayload()` 返回 `EventPayload`，
其中 `business` 保存业务 JSON，`image_content` 保存已编码的原生图片数组：

```cpp
EventPayload buildPayload() override
{
  auto payload = EventNodeBase::buildPayload();
  payload.business["data"] = {{"action", getInput<std::string>("action").value()},
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
colcon build --packages-up-to vln_behavior_tree --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select vln_behavior_tree --event-handlers console_direct+
colcon test-result --verbose
```

测试使用独立 ROS domain 的本地订阅者，覆盖动态插件加载、消息格式、图片、
黑板输入、自定义元数据及派生节点扩展。规划见 `docs/BEHAVIOR_TREE_PLAN.md`。
