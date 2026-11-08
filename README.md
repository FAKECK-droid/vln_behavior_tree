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

## 原生 GetImage 取图节点

`GetLatestColorImageService` 直接继承 Nav2 的 `BtServiceNode<rgbd_image_queue::srv::GetImage>`。
加载 `vln_get_latest_color_image_service_bt_node`，服务默认 `/rgbd_image_queue/get_image`。
operation 和双向 image_content 与工具版相同，支持 append、prepend、replace N 和 clear。
两种节点共用图片集合操作；clear 清空黑板中的集合，其余操作发送 LATEST_COLOR、detail=0。
原生压缩图字节直接写入 ImageContent，同时保留采样时间和服务返回的图像元数据。
服务等待默认 5000 毫秒，沿用 Nav2 黑板的服务等待配置。

```xml
<GetLatestColorImageService operation="append"
                            image_content="{pictures}"/>
```

## 八图环视事件

加载 `vln_submit_surrounding_images_bt_node`，使用 `SubmitSurroundingImages`。
节点读取 8 张 JPEG/PNG 图片，等比例缩小至网格内，留边居中，组成 3×3 图片。
第 1 张位于正上方，后续按 direction 排列，中央显示当前 map yaw 数值及方向箭头。
较小的源图片保留原尺寸。格子上的 1～8 为输入图片序号。

| 新增输入 | 默认值 | 含义 |
| --- | --- | --- |
| direction | counterclockwise | clockwise 顺时针，counterclockwise 逆时针 |
| tile_size | 256 | 每格边长，单位像素；输出边长为其三倍 |

沿用 EventNodeBase 的 image_content、event、message、pose、time_stamp 和 topic 端口。
image_content 作为输入保留原来的 8 张图。中央 yaw 使用基类 readPose() 获取的位姿，
默认读取 map → base_link 的最新 TF，也可沿用基类 pose 输入；图片标注与事件 pose 共用该快照。
中央 yaw 以角度显示，data.yaw 保存弧度值。TF 暂不可用时显示 unavailable，并继续上报图片。
默认输出 768×768 JPEG，质量 85；事件 content 中包含业务文本和一张合成图。
images[0] 直接记录 layout、direction、view_count、width、height 等元数据。
data.yaw 保存弧度值，data.views 按原图顺序保存各图的自定义元数据、采样信息及
source_index（从 0 开始）、row、column，便于关联原始观测。

顺时针布局：

```text
8  1  2
7 yaw 3
6  5  4
```

逆时针布局：

```text
2  1  8
3 yaw 7
4  5  6
```

```xml
<SubmitSurroundingImages name="ReportSurroundings"
                         event="surroundings"
                         image_content="{pictures}"
                         direction="counterclockwise"
                         tile_size="256"/>
```

合成图由派生类 buildPayload() 准备，再调用基类 buildPayload(images, pose) 构建公共业务字段、
完成一次 Base64 编码并发布。输入图片数量、解码或参数出错时返回 FAILURE。

## 数值表达式更新导航目标

加载 `vln_update_goal_bt_node`，使用同步节点 `UpdateGoal` 更新导航树中的目标：

```xml
<UpdateGoal goal="{goal}"
            x="x + 1.0"
            y="3.0"
            yaw="yaw + pi / 2"/>
```

| 端口 | 方向 / 类型 | 含义 |
| --- | --- | --- |
| goal | 双向 PoseStamped | 读取并写回导航目标 |
| x、y、z | 可选输入，字符串 | 位置计算表达式，空值保留原坐标 |
| yaw | 可选输入，字符串 | 偏航角表达式，空值保留原姿态 |
| variables | 可选输入，GoalVariables | std::map<std::string, double> 自定义数值变量 |

表达式中的 x/y/z/yaw 来自本次执行前的目标，所有字段基于同一份快照计算。
x/y/z 沿目标 frame_id 所在坐标系取值，单位米；yaw 单位弧度，逆时针为正。
内置变量 x/y/z/yaw/pi 由节点提供，variables 为表达式提供其他自定义数值。
支持四则运算、括号以及 sin、cos、atan2、sqrt、abs、min、max 等函数。
`^` 表示乘方，按右结合计算，例如 `2^3^2=512`，`-2^2=-4`。

```cpp
#include <vln_behavior_tree/plugins/actions/update_goal.hpp>

blackboard->set("goal_variables", vln_behavior_tree::GoalVariables{
  {"step", 0.5}});
```

```xml
<UpdateGoal goal="{goal}"
            variables="{goal_variables}"
            x="x + step * cos(yaw)"
            y="y + step * sin(yaw)"/>
```

上述例子将目标沿目标自身朝向前移 step 米。全部表达式计算成功后统一写回目标，
更新 yaw 时保留原 roll/pitch 并生成归一化四元数，frame_id 保持原值。
有数值变化时使用宿主 ROS 时钟更新时间戳；空表达式或结果相同时保留整个原始目标。
输入或表达式出错、结果非有限时返回 FAILURE 并记录日志，原目标保持原值。
宿主黑板提供 node，与其他 Nav2 节点一致。

每次 tick 执行一次运算，`x="x + 1"` 连续执行三次会累计增加 3 米。
可以把该节点放在行为树中需要调整目标的分支，后续 ComputePathToPose 使用更新后的 goal。

表达式解析器随包附带，使用 [TinyExpr](https://github.com/codeplea/tinyexpr)，
版本和 zlib 许可证见 `third_party/tinyexpr/UPSTREAM.md`、`LICENSE`。
解析器作为私有静态依赖编入节点库，构建时使用包内源码。

## 导航事件节点

加载 `vln_navigation_event_bt_node`，使用 `NavigationEvent` 提交导航过程事件。
沿用 EventNodeBase 的 event、message、pose、time_stamp、topic、image_content。
新增输入默认读取导航树已有的同名黑板变量：

| 输入 | 类型 | 写入业务 data |
| --- | --- | --- |
| goal | geometry_msgs::msg::PoseStamped | 当前单目标位姿 |
| goals | std::vector<geometry_msgs::msg::PoseStamped> | 当前多目标序列 |
| number_recoveries | int | 导航恢复次数 |

尚未提供的导航黑板项会省略。位姿使用 frame_id、x/y/z、yaw 和可选真实 UTC time_stamp。
目标与恢复次数统一在 buildPayload() 中转换；顶层 pose 表示事件发生时的小车位姿。
行为树通过 event 和节点所在分支决定事件含义，也可按需重映射上述输入。

```xml
<NavigationEvent name="ReportNavigationRecovery"
                  event="navigation_recovery"
                  message="开始执行恢复动作。"/>
```

两棵默认导航树已经接入以下事件：

| event | 上报时机 |
| --- | --- |
| navigation_started | 每次导航开始 |
| navigation_recovery | 规划恢复、控制恢复或系统恢复开始，origin 和 message 标识具体分支 |
| navigation_completed | 导航成功完成 |
| navigation_failed | 重试耗尽或恢复失败，导航最终返回 FAILURE |

NavigationEvent 提交目标、恢复次数和当前位姿。规划路径保留在导航黑板中供规划与跟踪使用。
各上报节点由 ForceSuccess 包装，导航结果继续由原有导航分支决定；
最终失败分支通过 AlwaysFailure 保持 FAILURE。
使用这两棵默认树时，将 vln_navigation_event_bt_node 加入 Nav2 的 plugin_lib_names。

## lookaround 行为树

`behavior_trees/lookaround.xml` 以 `LookAround` 为入口，使用调用方传入的 goal。
输入 goal 的位置由调用方设置为当前小车位置，树中保留该坐标并更新朝向。

1. clear 清空黑板中的 image_content。
2. 循环 8 次：获取最新彩色图并 append → UpdateGoal 将 yaw 增加 pi/8 → 执行单目标导航子树。
3. 循环完成后，SubmitSurroundingImages 提交一张环视合成图，事件为 lookaround_completed。

每次增加 pi/8，8 次累计增加 pi。采图发生在每次目标朝向更新之前。
子树运行期间使用 Sequence 保持当前步骤，图像采集或子树失败时结束该流程。

| 文件 | 入口 ID | 功能 |
| --- | --- | --- |
| vln_nav2goal.xml | VlnNav2Goal | 使用 goal 的单目标导航 |
| vln_navthroughgoals.xml | VlnNavThroughGoals | 使用 goals 的多目标导航 |
| lookaround.xml | LookAround | 获取八张图片并依次更新朝向 |

lookaround 通过 ros_pkg include 定位包内 vln_nav2goal.xml，子树采用 Humble / BT.CPP 3 的
`__shared_blackboard="true"`，复用 goal、path 和 Nav2 宿主配置。
这种 include 方式也适用于 Humble 读取 XML 文本后创建行为树的加载流程。

在原有 Nav2 plugin_lib_names 中追加：

```yaml
- vln_get_latest_color_image_service_bt_node
- vln_update_goal_bt_node
- vln_submit_surrounding_images_bt_node
- vln_navigation_event_bt_node
```

NavigateToPose 请求的 behavior_tree 指向安装后的 share/vln_behavior_tree/behavior_trees/lookaround.xml。
lookaround 复用导航子树，每次转向会产生导航开始及终态事件，循环完成后另行提交 lookaround_completed。

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
