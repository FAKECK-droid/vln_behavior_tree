# VLN Event 上报节点规划

更新日期：2026-11-08。通用事件基类适配当前 Agent 业务 JSON 规范。

## 1. 职责

Event 节点从输入构建机器事件，发布给 EventManager，供模型决策参考。
行为树决定上报时机和后续动作；观测和动作状态由其他节点准备，通过黑板或端口传入；缺省位姿由基类读取 TF。

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
| pose | Pose2D，可选 | 对应的小车位姿；为空时读取 map → base_link 最新 TF |
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
pose 未提供、尚未填值或 frame_id 为空时，读取 map → base_link 的最新 TF，
优先复用 Nav2 黑板的 tf_buffer，独立使用时创建监听器。TF 暂不可用时仍发布其余事件信息。
事件顶层位姿表示上报时的位置，图片中的 pose 保留提供方填写的采样位姿。

`ImageContent::Image` 保存 JPEG/PNG 文件字节、mime_type、自定义 metadata map，
并支持可选的 time_stamp 和 pose。图片元数据形式为：

```json
{
  "time_stamp": "2026-11-07T02:30:00.080Z",
  "label": "door",
  "score": 0.97
}
```

每张图片按顺序对应一个 images 元素和一个原生 input_image。
metadata 内容由提供方定义，map 键值直接写入图片描述；同名的非空采样字段优先。
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

- behavior_trees/vln_nav2goal.xml
- behavior_trees/vln_navthroughgoals.xml

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
2026-11-08：新增取图节点，图片自定义字段平铺至描述，事件缺省位姿从 TF 获取。

## 7. 工具调用基类

CallToolNodeBase 继承 Nav2 的 BtServiceNode<vln_agent::srv::ToolCall>，
复用 Agent 已实现的 /vln/tool_call 服务。其通用插件名称为 CallTool。

- 输入 tool 和 arguments；派生类可增加普通类型端口并通过 buildArguments() 构建参数。
- 每次开始执行生成独立调用 ID，向服务提交一个标准 completed/function_call 请求。
- 等待期间返回 RUNNING，使用 Nav2 服务超时配置，默认等待工具完整结果 5000 毫秒。
- 将返回的业务 payload 和原生图片还原为 vln_agent::ToolResult，通过 result 端口交付。
- 默认按 payload.ok 返回状态；派生类可通过 onResult() 组织自己的结果处理。
- 结束、异常和 halt 清理本地挂起请求，已提交工具继续由 Agent 处理。

公共头文件为 include/vln_behavior_tree/call_tool_node_base.hpp，
实现文件为 src/call_tool_node_base.cpp，导出库为 vln_call_tool_bt_node。
验证覆盖请求格式、独立调用 ID、类型化参数、业务和图片原样保留、工具及服务错误、
超时后重试、halt、派生结果处理，以及真实 ToolManager 服务的协议对接。

## 8. 最新图像节点

GetLatestColorImage 继承 CallToolNodeBase，通过 ToolManager 服务调用 capture_image，
参数为 mode=latest、detail=0。业务端口为 operation 和双向 image_content。

- append / prepend：将新图添加至集合尾部 / 头部。
- replace N：替换从 0 开始的第 N 项，-1 使用最后一项。
- clear：直接写回空集合并返回 SUCCESS。

取图操作返回 RUNNING 直至收到结果，将服务中 Base64 图片还原为文件字节，
将图片描述中的自定义元数据、采样时间和采样位姿还原到 ImageContent。
取图和转换成功后更新同一个黑板变量。异常返回 FAILURE，保留集合原值。
EventNodeBase 再按最新格式构建事件，图片 metadata map 的键值直接位于 images 元素中。

插件文件为 plugins/tools/get_latest_color_image.cpp，
库名为 vln_get_latest_color_image_bt_node，注册名称为 GetLatestColorImage。


## 9. 原生取图与环视事件

GetLatestColorImageService 直接使用 Nav2 BtServiceNode<GetImage> 调用原生取图服务，
沿用 operation 和双向 image_content，固定请求 LATEST_COLOR、detail=0。
与工具版共用 ImageOperation 和图片端口读写函数，保持集合操作一致。
服务结果保留压缩图文件字节、原有元数据和真实采样时间，供 Event 节点统一编码。

SubmitSurroundingImages 继承 EventNodeBase，读取八张图、direction 和 tile_size，yaw 从基类位姿获取。
第一张放正上方，按指定方向排列到 3×3 外圈，中央显示 yaw，合成为一张 JPEG。
图片等比例缩小，居中留边；输出边长为 tile_size 的三倍，默认 768 像素。
原始集合保留，事件 data.views 保存每张图的原始信息和网格位置。
派生类通过 readPose() 与 buildPayload(images, pose) 复用同一份位姿及公共字段，只发布合成图。

导出插件：

- vln_get_latest_color_image_service_bt_node / GetLatestColorImageService
- vln_submit_surrounding_images_bt_node / SubmitSurroundingImages


## 10. 目标数值更新节点

UpdateGoal 继承 BT::SyncActionNode，goal 为双向 geometry_msgs::msg::PoseStamped。
x、y、z、yaw 为可选数值表达式，variables 为 std::map<std::string, double>。
每次执行读取一份原始目标，所有表达式使用该快照中的 x/y/z/yaw 和常量 pi，
并绑定调用方提供的自定义变量。解析与计算完成后统一写回。

位置按目标所在坐标系计算，yaw 使用弧度；更新 yaw 时保留 roll/pitch，生成归一化四元数。
frame_id 保持原值，目标有数值变化时用 ROS 时钟更新时间戳。
空表达式或无数值变化时保留原目标。失败返回 FAILURE，原目标保持原值。
相对运算在每次 tick 时累计执行，行为树决定目标调整时机。

TinyExpr 解析器以固定版本随包提供，支持基础数学运算和函数，并补充 min/max。
插件库为 vln_update_goal_bt_node，注册名为 UpdateGoal。
验证覆盖快照语义、自定义变量、数学函数、原子写回、姿态、时间戳与下游目标读取。


## 11. 导航事件与 lookaround

NavigationEvent 复用事件基类，默认从现有导航黑板读取 goal、goals、number_recoveries。
存在的数据写入 data，缺省项省略；event 与 message 沿用基类端口。
目标位姿按公共字段形式序列化，保留坐标系、x/y/z、yaw 和真实采样时间。

两棵导航树采用独立入口 ID：VlnNav2Goal（单目标）与 VlnNavThroughGoals（多目标）。
lookaround.xml include 单目标树，使用共享黑板复用调用方提供的当前位姿 goal。
清空图片集合后循环八次：append 最新图像 → yaw 增加 pi/8 → 调用单目标子树。
最后提交 SubmitSurroundingImages。中央 yaw 与事件 pose 从基类 readPose() 的同一快照获取。
TF 暂不可用时仍提交图片，中央标记 unavailable。

验证覆盖默认黑板输入、序列化、图片兼容、TF 位姿复用、文本 include 加载、
八次循环顺序、坐标保持、等待过程、提前失败与 halt。导航叶节点使用本地替身，
实际图片服务节点、目标更新节点、Nav2 控制节点和事件提交节点参与集成测试。


## 12. 默认导航树事件接入

单目标、多目标树均在开始、规划恢复、控制恢复、系统恢复和终态分支放置 NavigationEvent。
event 使用 navigation_started、navigation_recovery、navigation_completed、navigation_failed；
origin 和 message 描述具体恢复分支。事件保留目标与恢复次数，path 用于树内规划和跟踪。
NavigationEvent 的 path 端口与路径序列化已移除。

上报节点通过 ForceSuccess 保持辅助上报语义；导航最终失败分支使用 AlwaysFailure。
原有规划频率、恢复次数和恢复动作顺序保持不变。
验证覆盖两棵树的成功、规划/控制/系统恢复、重试耗尽、上报失败和 lookaround 子树事件。
