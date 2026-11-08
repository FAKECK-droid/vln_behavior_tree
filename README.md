# vln_behavior_tree

VLN 导航行为树、Event 上报及 Agent 工具调用节点。`SubmitEvent` 从输入构建事件并发布给
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
| `pose` | 最新 TF | 黑板中的 `Pose2D`；为空时读取 `map → base_link` |
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
  "pose": {"frame_id": "map", "x": 1.2, "y": 2.3, "yaw": 0.5},
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
省略 pose、黑板值尚未填入或 Pose2D.frame_id 为空时，读取 `map → base_link` 的最新 TF。
优先复用 Nav2 黑板中的 `tf_buffer`；独立使用时基类创建 TF 监听器。
TF 暂不可用时记录日志，事件仍正常上报，稍后执行会再次查询。
TF 时间戳为非零真实 ROS 时间时写入 pose.time_stamp；仿真时省略该 UTC 字段。

## 图片输入

`image_content.hpp` 提供 `ImageContent`，其 `images` 数组的每项包含：

- `bytes`：JPEG、PNG 等图片文件字节，`std::vector<unsigned char>`。
- `metadata`：自定义 `std::map<std::string, MetadataValue>`；值支持字符串、布尔、int64、uint64 和 double。
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
  "label": "door",
  "score": 0.97,
  "sequence": 7,
  "usable": true
}
```

可选的图片位姿放在描述的 `pose` 中。metadata map 的键值直接写入该图片描述；
同名的非空 Image.time_stamp 和 Image.pose 优先使用采样字段。空描述为 `{}`。
省略图片端口或传入空集合时发送纯文字事件。发布前复用 Agent 的图片数量检查，确保 `images` 与原生图片逐张对应。

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

## 工具调用节点

`CallToolNodeBase` 继承 Nav2 的 `BtServiceNode<vln_agent::srv::ToolCall>`，
注册为通用 `CallTool` 节点。将 `vln_call_tool_bt_node` 追加到 Nav2 的
`plugin_lib_names`，并启动已提供 ToolCall 服务的 Agent：

```xml
<CallTool name="ReadLocalMap"
          tool="get_surroundings"
          arguments='{"costmap":"local","range_m":10.0}'
          result="{surroundings}"/>
```

| 端口 | 方向 | 默认值 | 含义 |
| --- | --- | --- | --- |
| tool | 输入 | 由调用方提供 | Agent 中启用的工具函数名 |
| arguments | 输入 | {} | 参数 JSON 字符串，可写在 XML 或通过黑板传入 |
| service_name | 输入 | /vln/tool_call | 沿用 Nav2 的服务端口 |
| server_timeout | 输入 | 5000 | 等待本次完整工具结果的时限，单位毫秒 |
| result | 输出 | 本次开始时清空 | vln_agent::ToolResult，含 payload 和 images |

基类在每次请求开始时读取输入、生成独立的 robot_bt_ 调用 ID，并构建 Agent 服务
所需的 completed/function_call 请求。等待服务响应期间返回 RUNNING；取得结果后，
`result.payload` 保留 source/time_stamp/ok/data/pose/images 等业务信息，
`result.images` 保留原生 input_image 图片数组及已有的 Base64 内容。

默认以 payload.ok 决定 SUCCESS/FAILURE。工具失败结果仍写入 result，
其 code/message 可供后续行为树读取；data.status 表示导航物理任务状态。
服务报错、参数构建或响应解析异常返回 FAILURE。每次新请求清空上次输出；
结束、超时和 halt 会清理本地挂起请求及响应正文。
halt 或超时结束本地等待，已提交工具仍由 Agent 处理。

与 Nav2 其他服务节点一样，宿主黑板提供 node、bt_loop_duration、server_timeout 和
wait_for_service_timeout，时间配置使用 std::chrono::milliseconds。

### 派生工具节点

派生类可通过 buildArguments() 从普通类型端口构建参数，通过 onResult() 处理结果。
例如，让上游只提供地图名称和数值范围：

```cpp
class GetSurroundingsNode : public CallToolNodeBase
{
public:
  using CallToolNodeBase::CallToolNodeBase;

  static BT::PortsList providedPorts()
  {
    auto ports = CallToolNodeBase::providedPorts();
    ports.at("tool") = BT::InputPort<std::string>(
      "tool", "get_surroundings", "工具名").second;
    ports.insert(BT::InputPort<std::string>("costmap", "local", "地图来源"));
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
```

自定义结果状态时重写 `BT::NodeStatus onResult(const ToolResult & result)`；
调用该方法前，原始结果已写入输出端口。派生插件链接
`vln_behavior_tree::vln_call_tool_bt_node`，包含 `vln_behavior_tree/call_tool_node_base.hpp`。

## 最新彩色图节点

加载 `vln_get_latest_color_image_bt_node` 后，使用 `GetLatestColorImage`。
节点通过 ToolManager 的 `/vln/tool_call` 服务调用 `capture_image`，固定参数为
`mode=latest, detail=0`。图像结果转换为普通 C++ 的 ImageContent，供事件节点继续使用。

| 业务端口 | 方向 | 默认值 | 含义 |
| --- | --- | --- | --- |
| operation | 输入 | append | 图片集合操作 |
| image_content | 双向 | 首次为空集合 | 读取和写回同一个黑板变量 |

沿用基类的 service_name 和 server_timeout 服务配置端口。

| operation | 操作 |
| --- | --- |
| append | 将新图添加到数组末尾 |
| prepend | 将新图添加到数组头部 |
| replace 0 | 用新图替换第一张 |
| replace -1 | 用新图替换最后一张 |
| replace N | 替换从 0 开始的第 N 项 |
| clear | 清空集合，直接返回 SUCCESS |

append/prepend 可从尚未初始化的黑板变量开始；replace 使用已有数组中的索引。
取图、解析或索引出错时返回 FAILURE，集合保留原值。clear 在本地写回空集合。
图片字节通过共用 Base64 解码函数还原，采样时间和自定义元数据一起保留；
提交事件时由 buildPayload() 编码并将元数据直接填入 images 对应元素。

```xml
<Sequence>
  <GetLatestColorImage operation="append" image_content="{pictures}"/>
  <SubmitEvent name="ReportObservation" image_content="{pictures}"/>
  <GetLatestColorImage operation="clear" image_content="{pictures}"/>
</Sequence>
```

专用工具节点可重写基类 toolName() 返回固定工具名，并通过 buildArguments() 构建参数。

## 编译与检查

使用具有统一业务封装接口的当前 `vln_agent` 版本，在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vln_behavior_tree --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select vln_behavior_tree --event-handlers console_direct+
colcon test-result --verbose
```

测试使用独立 ROS domain 的本地订阅者和工具服务，覆盖动态插件加载、统一业务格式、
采样时间、位姿、自定义图片元数据、派生节点扩展、工具响应、超时/停止和实际 Agent 服务对接。
规划见 `docs/BEHAVIOR_TREE_PLAN.md`。
