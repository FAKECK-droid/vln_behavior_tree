# VLN 行为树节点与导航流程

## 1. 功能与接入

`vln_behavior_tree` 提供 ROS 2 Humble / BehaviorTree.CPP 3 插件，连接 Nav2 导航、图像获取与 Agent 事件输入。

```text
Nav2 黑板 / 节点端口
  ├─ Event 节点 ─ /vln/robot_event ─ EventManager ─ 模型输入
  ├─ CallTool 节点 ─ /vln/tool_call ─ ToolManager ─ 工具结果
  └─ 原生图像服务节点 ─ /rgbd_image_queue/get_image ─ ImageContent
```

节点通过黑板的 `node` 取得宿主 ROS 节点。服务节点复用 Nav2 的执行与服务等待配置，事件节点可复用黑板中的 `tf_buffer`。

| 注册名称 | 插件库 |
| --- | --- |
| `SubmitEvent` | `vln_submit_event_bt_node` |
| `CallTool` | `vln_call_tool_bt_node` |
| `GetLatestColorImage` | `vln_get_latest_color_image_bt_node` |
| `GetLatestColorImageService` | `vln_get_latest_color_image_service_bt_node` |
| `SubmitSurroundingImages` | `vln_submit_surrounding_images_bt_node` |
| `NavigationEvent` | `vln_navigation_event_bt_node` |
| `UpdateGoal` | `vln_update_goal_bt_node` |

这些库按使用情况加入 `bt_navigator.ros__parameters.plugin_lib_names`。包内的 `vln_image_content_utils` 是图片集合辅助库，由对应插件链接。

## 2. 黑板数据

### 2.1 Pose2D

[pose_2d.hpp](../include/vln_behavior_tree/pose_2d.hpp) 定义二维位姿：

```cpp
struct Pose2D
{
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string time_stamp{};
};
```

x/y 使用米，yaw 为相对坐标系正 x 轴的逆时针弧度角。`time_stamp` 为可选的 UTC 毫秒采样时间，格式为 `YYYY-MM-DDTHH:mm:ss.SSSZ`。

### 2.2 ImageContent

[image_content.hpp](../include/vln_behavior_tree/image_content.hpp) 定义图片集合。每张 `Image` 包含：

| 成员 | 内容 |
| --- | --- |
| `bytes` | JPEG、PNG 等图片文件的二进制字节 |
| `mime_type` | 媒体类型，默认 `image/jpeg` |
| `metadata` | 自定义键值，支持字符串、布尔、int64、uint64、double |
| `time_stamp` | 可选图片采样时间 |
| `pose` | 可选采图时小车位姿 |

`ImageContent::images` 按采集或集合操作的顺序保存图片。提供方准备文件字节和普通 C++ 元数据，Event 节点将其编码为模型输入。

```cpp
vln_behavior_tree::ImageContent pictures;
vln_behavior_tree::ImageContent::Image image;
image.bytes = jpeg_bytes;
image.metadata["label"] = std::string("door");
image.time_stamp = "2026-11-09T06:00:00.000Z";
pictures.images.push_back(std::move(image));
blackboard->set("pictures", pictures);
```

## 3. EventNodeBase 与 SubmitEvent

公共接口见 [event_node_base.hpp](../include/vln_behavior_tree/event_node_base.hpp)。

`EventNodeBase` 继承 `BT::SyncActionNode`，直接注册名称为 `SubmitEvent`。一次执行构建业务 JSON、图片描述和原生图片，发布后返回 `SUCCESS`；构建或发布异常写入 ROS 日志并返回 `FAILURE`。

### 3.1 输入端口

| 端口 | 类型 | 缺省值或行为 |
| --- | --- | --- |
| `topic` | string | `/vln/robot_event` |
| `event` | string | `status` |
| `message` | string | 空；有内容时写入 `data.message` |
| `time_stamp` | string | 使用当前 UTC 毫秒时间 |
| `pose` | `Pose2D` | 读取 `map → base_link` 最新 TF |
| `image_content` | `ImageContent` | 空集合 |

显式输入的 pose 优先；frame_id 为空或端口缺省时查询 TF。节点先使用 Nav2 黑板的 `tf_buffer`，独立使用时创建监听器。TF 暂不可用时，事件携带其余业务字段。

### 3.2 发布格式

话题类型为 `std_msgs/msg/String`，QoS 为 reliable、volatile、KeepLast(10)。`data` 保存一条原生消息：

```json
{
  "role": "user",
  "content": [
    {
      "type": "input_text",
      "text": "{\"source\":\"robot\",\"time_stamp\":\"2026-11-09T06:00:00.000Z\",\"origin\":\"ReportArrival\",\"event\":\"navigation_completed\",\"data\":{\"message\":\"已到达目标。\"}}"
    }
  ]
}
```

其中业务字段 `source/time_stamp/data` 来自 Agent 的公共封装，`origin` 使用节点实例名，`event` 来自端口。`SUCCESS` 表示已向 ROS 发布器提交该条消息。

有图片时，业务 JSON 中的 `images` 和后续原生 `input_image` 按顺序对应。`Image::metadata` 的键值平铺至图片描述，有值的采样字段 `time_stamp/pose` 写入同一描述：

```json
{
  "images": [
    {
      "label": "door",
      "time_stamp": "2026-11-09T06:00:00.000Z",
      "pose": {
        "frame_id": "map",
        "x": 1.2,
        "y": 2.3,
        "yaw": 0.5
      }
    }
  ]
}
```

图片内容编码为 `data:<mime_type>;base64,...`，`detail` 为 `auto`。事件顶层 pose 表示上报时小车位置，图片 pose 表示提供方记录的采样位置。

```xml
<SubmitEvent name="ReportArrival"
             event="navigation_completed"
             message="已到达目标。"
             image_content="{pictures}"/>
```

### 3.3 派生接口

派生类通过 `providedPorts()` 合并基类端口，并重写 `buildPayload()`：

```cpp
EventPayload buildPayload() override
{
  auto event = EventNodeBase::buildPayload();
  event.business["data"]["distance_remaining_m"] = 2.4;
  return event;
}
```

`EventPayload::business` 保存业务 JSON，`image_content` 保存已编码的原生图片数组。基类还提供 `buildPayload(images)` 和 `buildPayload(images, pose)`，供派生类使用自己准备的图片及同一份位姿快照。`readPose()` 可单独获取该快照。

工具建议可写入 `business["instruction"]`。EventManager 将消息交给模型，由模型生成实际函数调用。

## 4. CallToolNodeBase 与 CallTool

公共接口见 [call_tool_node_base.hpp](../include/vln_behavior_tree/call_tool_node_base.hpp)。

基类继承 `nav2_behavior_tree::BtServiceNode<vln_agent::srv::ToolCall>`，注册为 `CallTool`。

| 端口 | 方向与类型 | 用途 |
| --- | --- | --- |
| `tool` | 输入 string | Agent 中启用的工具名 |
| `arguments` | 输入 string | 参数 JSON 对象，默认 `{}` |
| `result` | 输出 `vln_agent::ToolResult` | 业务 payload 和原生图片 |
| `service_name` | 输入 string | 默认 `/vln/tool_call` |
| `server_timeout` | 输入毫秒 | 默认 5000，等待本次工具结果 |

```xml
<CallTool tool="get_surroundings"
          arguments='{"costmap":"local","range_m":10.0}'
          result="{surroundings_result}"/>
```

`on_tick()` 为每次调用创建 `robot_bt_...` ID，把工具名和参数包装成服务请求的 `completed/function_call` 对象。服务等待期间返回 `RUNNING`。

`on_completion()` 读取 `outputs_json`，恢复业务 JSON 和原生图片，先写入 `result`，再调用 `onResult()`。缺省 `onResult()` 按 `payload.ok` 返回成功或失败。工具函数返回成功与物理导航任务结束分别由各自状态表示，例如启动导航的工具结果表示目标已被接受。

派生类可重写：

| 方法 | 用途 |
| --- | --- |
| `toolName()` | 提供固定或端口指定的工具名 |
| `buildArguments()` | 从类型化端口组织参数对象 |
| `onResult()` | 处理结果并决定节点返回状态 |

结束、异常和 `halt()` 会清理本地挂起请求与响应数据。已提交工具由 Agent 继续执行，动作取消通过对应取消工具发起。

服务中的 `function_call_output` 用于关联本次机器人调用。需要将结果交给模型时，调用方把观测数据或图片写入事件，再由 Event 节点发布。

## 5. 彩色图像获取

两种取图节点共用 `operation` 和双向 `image_content` 端口。`image_content` 绑定黑板变量，初始未填值时作为空集合读取。

| `operation` | 对图片集合的操作 |
| --- | --- |
| `append` | 添加到末尾，默认操作 |
| `prepend` | 添加到开头 |
| `replace N` | 替换第 N 项，索引从 0 开始 |
| `replace -1` | 替换最后一项 |
| `clear` | 清空黑板图片集合，立即返回 `SUCCESS` |

取图成功并完成转换后，节点将集合写回黑板。失败保留取图前的集合。采样时间按真实 ROS 时间转换为 UTC；仿真采样时间省略。

### 5.1 GetLatestColorImage

该节点派生自 CallToolNodeBase，调用 Agent 的 `capture_image`，固定参数为 `mode=latest/detail=0`。结果图片通过 Base64 解码恢复为文件字节；`images[0].metadata` 转成普通 C++ 键值，采样时间和可选 pose 分别恢复到对应成员。

```xml
<GetLatestColorImage operation="append"
                     image_content="{pictures}"/>
```

业务端口为 `operation/image_content`，服务名与超时沿用工具调用基类。

### 5.2 GetLatestColorImageService

该节点直接继承 `BtServiceNode<rgbd_image_queue::srv::GetImage>`，请求固定为 `LATEST_COLOR/detail=0`。默认服务名 `/rgbd_image_queue/get_image`，结果等待超时为 5000 毫秒。

```xml
<GetLatestColorImageService operation="append"
                            image_content="{pictures}"/>
```

服务返回的压缩图片直接存入 `ImageContent`，同时保存 frame_id、格式、配对 ID、索引、队列数量和年龄信息。两个节点均通过公共 `ImageOperation` 完成集合操作。

## 6. SubmitSurroundingImages

该节点继承 EventNodeBase，将八张图片合成为一张环视 JPEG。

| 端口 | 含义 |
| --- | --- |
| `image_content` | 八张输入图片，按集合顺序排列 |
| `direction` | `clockwise` 或 `counterclockwise`，默认后者 |
| `tile_size` | 每格边长，默认 256 像素 |
| Event 基类端口 | 事件类型、说明、话题、时间和位姿 |

第一张图片放在 3×3 网格正上方，其余沿指定方向排入外圈。图片等比例缩小、居中留边，并标注从 1 开始的序号。中央显示本次事件位姿中的 yaw 及坐标系，角度显示单位为度。

输出边长为 `3 × tile_size`，默认 768；JPEG 质量为 85。中央标注和事件顶层 pose 使用同一份 `readPose()` 快照。位姿缺省时，中央显示 `unavailable`。

```xml
<SubmitSurroundingImages name="ReportLookAround"
                         event="lookaround_completed"
                         image_content="{pictures}"
                         direction="counterclockwise"
                         tile_size="256"/>
```

原始图片集合继续保存在黑板。发布的事件携带一张合成图：

- `images[0]` 描述合成图，包含 `layout/direction/view_count/width/height/time_stamp`。
- `data.views` 按原图顺序保存自定义元数据、采样信息、`source_index` 和网格 `row/column`。
- 位姿可用时，`data.yaw` 与顶层 pose 的 yaw 相同。

`source_index/row/column` 均从 0 开始。独立使用时事件类型来自 `event` 端口，默认值继承基类的 `status`。

## 7. UpdateGoal

该同步节点读取导航目标，使用数值表达式更新位置和 yaw。

| 端口 | 方向与类型 | 含义 |
| --- | --- | --- |
| `goal` | 双向 `geometry_msgs::msg::PoseStamped` | 当前目标 |
| `x/y/z/yaw` | 输入 string | 各字段表达式，空值保留原字段 |
| `variables` | 输入 `GoalVariables` | `std::map<std::string,double>` 自定义变量 |

```xml
<UpdateGoal goal="{goal}"
            x="x + 0.5 * cos(yaw)"
            y="y + 0.5 * sin(yaw)"
            yaw="yaw + pi / 4"/>
```

所有表达式读取同一份原始目标。内置变量为 `x/y/z/yaw/pi`，这些名称的值来自目标和常量；自定义变量通过 `variables` 注入。表达式由随包提供的 TinyExpr 求值，支持算术、右结合幂运算和 `sin/cos/atan2/sqrt/abs/min/max` 等函数。

位置表达式使用目标 frame_id 所在坐标系，yaw 使用弧度。修改 yaw 时保留 roll/pitch，生成归一化四元数。完成求值后统一写回；有数值变化时使用当前 ROS 时间更新时间戳。空表达式或数值相同的结果保留目标原值。计算异常返回 `FAILURE`，目标原值保留。

相对表达式在每次执行时累计，行为树决定其执行位置和频率。

## 8. NavigationEvent

该事件节点从 Nav2 黑板读取导航目标和恢复次数。附加端口的默认映射与端口同名：

| 端口 | 类型 | 默认映射 |
| --- | --- | --- |
| `goal` | `geometry_msgs::msg::PoseStamped` | `{goal}` |
| `goals` | `std::vector<geometry_msgs::msg::PoseStamped>` | `{goals}` |
| `number_recoveries` | int | `{number_recoveries}` |

存在的值进入 `data.goal/data.goals/data.number_recoveries`。目标位姿包含 frame_id、x/y/z、yaw 和可用的 UTC 采样时间。节点同时继承 Event 基类的说明、图片和位姿端口。

```xml
<NavigationEvent name="ReportPoseNavigationCompleted"
                 event="navigation_completed"
                 message="已到达导航目标。"/>
```

规划路径留在树内黑板，用于规划和跟踪；上报数据由目标、恢复次数及事件公共字段组成。

## 9. 导航树

| 文件 | 入口 ID | 主要输入 |
| --- | --- | --- |
| `behavior_trees/vln_nav2goal.xml` | `VlnNav2Goal` | 黑板 `goal` |
| `behavior_trees/vln_navthroughgoals.xml` | `VlnNavThroughGoals` | 黑板 `goals` |
| `behavior_trees/lookaround.xml` | `LookAround` | 当前机器人位姿作为 `goal` |

### 9.1 单目标与多目标导航

两棵导航树使用 `PipelineSequence` 组织规划与跟踪，外层 `RecoveryNode` 重试次数为 6。规划和跟踪分支各有一次上下文恢复。

- 单目标使用 `ComputePathToPose`，规划频率为 1 Hz。
- 多目标使用 `RemovePassedGoals` 和 `ComputePathThroughPoses`，规划频率为 0.333 Hz，经过点的移除半径为 0.7 米。
- 规划恢复清理全局代价地图，跟踪恢复清理局部代价地图。
- 系统恢复使用 RoundRobin，依次安排局部/全局清图、旋转 1.57 弧度、等待 5 秒、后退 0.30 米（0.05 米/秒）。

事件点位于开始、规划恢复、跟踪恢复、系统恢复、成功和最终失败分支：

| 事件类型 | 时机 |
| --- | --- |
| `navigation_started` | 开始执行导航 |
| `navigation_recovery` | 进入某个恢复分支 |
| `navigation_completed` | 导航成功 |
| `navigation_failed` | 恢复耗尽后的最终失败 |

恢复阶段由节点实例名和 `data.message` 区分。事件使用 `ForceSuccess` 包装，事件成功或失败后继续对应导航分支；最终失败分支通过 `AlwaysFailure` 返回导航失败。

### 9.2 LookAround

环顾树通过 `ros_pkg` include 引用 `vln_nav2goal.xml`，使用 `__shared_blackboard="true"` 传递目标。执行顺序为：

1. 清空 `image_content`。
2. 重复八次：获取最新彩色图并 append，执行 `yaw + pi / 4`，运行单目标导航子树。
3. 执行 SubmitSurroundingImages，发布 `lookaround_completed`。

每次转向为 45°，八次合计一周。目标位置取自调用方提供的当前机器人位姿，各次更新保留其坐标。每次子导航上报相应导航事件，最终事件携带一张环视合成图。

Agent 的 `start_lookaround` 工具使用当前 TF 位姿构建 NavigateToPose 目标，并将 `behavior_tree` 指向此文件。取消通过同一 Action 的取消请求处理。

## 10. Nav2 集成与扩展

使用包内完整导航树时，Nav2 继续装载规划、跟踪和恢复节点所需的插件，并添加实际使用的 VLN 插件。环顾树所需的 VLN 库为：

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - vln_get_latest_color_image_service_bt_node
      - vln_update_goal_bt_node
      - vln_navigation_event_bt_node
      - vln_submit_surrounding_images_bt_node
```

以上为需要追加的 VLN 条目。NavigateToPose 的 `behavior_tree` 使用包安装目录中 XML 的绝对路径。

扩展事件节点时，合并基类端口，组织 `EventPayload::business["data"]`，通过公共图片集合和位姿快照构建模型输入。扩展工具节点时，提供工具名、类型化参数和结果处理。两类节点各自通过 ROS 话题和 ToolCall 服务接入 Agent。

源码组织：

```text
include/vln_behavior_tree/
  event_node_base.hpp
  call_tool_node_base.hpp
  pose_2d.hpp
  image_content.hpp
  image_content_utils.hpp
  plugins/{events,tools,services,actions}/
src/                              # 基类与图片集合操作
plugins/
  events/                         # NavigationEvent、SubmitSurroundingImages
  tools/                          # GetLatestColorImage
  services/                       # GetLatestColorImageService
  actions/                        # UpdateGoal
behavior_trees/
third_party/tinyexpr/
test/
```
