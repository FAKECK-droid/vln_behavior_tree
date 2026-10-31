# VLN Event 上报节点规划

日期：2026-10-31。第一部分 Event 节点基类已实现，后续再接入本包的两棵 Nav2 导航行为树。使用和派生方式见 `README.md`。

## 1. 节点职责

Event 节点只做一件事：**把输入数据组装成 Event，发布给现有 EventManager，供模型决策参考。**

行为树决定何时执行这个节点，以及上报后继续导航、恢复还是结束。动作状态、传感器数据和其他业务信息由外部准备，通过输入端口传入；Event 节点不自行获取这些数据，也不等待模型应答。

```text
其他节点准备数据 → 黑板 / 输入端口 → Event 节点 → /vln/robot_event
```

## 2. 简化构成

只设计一个 `EventNodeBase`，继承 `BT::SyncActionNode`，内部直接持有 ROS 发布器。

它本身即可注册为通用的 `SubmitEvent` 节点使用；有特殊数据组织需求时，再继承它扩展。第一版不拆分 Reporter、Sink、Codec 或运行时管理器。

每次执行的流程：

1. 调用 `buildPayload()`，根据输入数据构建业务 payload，并完成图片 Base64 编码与元数据转换。
2. 将返回的业务数据和已编码图片整体包装为现有 EventManager 接收的消息格式。
3. 发布到配置的话题，正常返回 `SUCCESS`；输入解析或发布发生异常时记录日志并返回 `FAILURE`。

`SUCCESS` 表示本次本地发布调用完成。节点不等待接收回执或模型处理结果。

重复上报、上报频率以及失败后是否重试，由行为树组合其他节点决定，Event 节点不额外维护这些策略。

## 3. 输入端口

| 端口 | 类型 | 用途 |
| --- | --- | --- |
| `topic` | string | 上报话题，默认 `/vln/robot_event` |
| `event` | string | 事件类型，默认 `status` |
| `text` | string | 事件说明文字 |
| `image_content` | `ImageContent`，可选 | 图片字节数组及完全由提供方定义的元数据 map |

动作状态、传感器等业务数据通过派生类定义的输入端口传入，统一在 `buildPayload()` 中组织为事件内容。

例如，`buildPayload()` 根据传入的动作名、状态和剩余距离生成：

```json
{
  "source": "robot",
  "time_stamp": "2026-10-31T06:00:00.000Z",
  "origin": "ReportNavigationFailure",
  "event": "error",
  "text": "路径跟随失败。",
  "data": {
    "action": "FollowPath",
    "state": "FAILURE",
    "distance_remaining_m": 2.4
  }
}
```

`buildPayload()` 同时填写公共字段：`source` 为 `robot`，`time_stamp` 默认使用当前 UTC 毫秒时间，`origin` 默认使用节点实例名。事件发生时间、位姿和图片元数据等按输入数据在该函数中组装。

`ImageContent` 包含 `std::vector<Image>`；每个 `Image` 保存图片文件字节
`std::vector<unsigned char>`、自定义元数据 `std::map<std::string, MetadataValue>` 和
`mime_type`。元数据值支持文字、布尔、整数和浮点数，键名由提供方决定。

`buildPayload()` 读取该对象，将完整元数据 map 按顺序转换为业务 `images`，
将字节编码为对应的 `input_image`。上游通过标准 C++ 容器填写数据。
图片数据结构定义见 `include/vln_behavior_tree/image_content.hpp`。

## 4. 基类接口草案

```cpp
class EventNodeBase : public BT::SyncActionNode
{
public:
  EventNodeBase(const std::string & name,
                const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

protected:
  // 构建业务数据，并编码图片及其自定义元数据。
  virtual EventPayload buildPayload();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_;
};
```

构造时从 Nav2 黑板的 `node` 获取宿主 ROS 节点，发布器按 `topic` 创建并复用。时间和 Base64 编码复用 `vln_agent_utils`。

`EventPayload` 包含业务 JSON `business` 和已编码图片数组 `image_content`。基类的 `buildPayload()` 统一读取和转换输入；`tick()` 将业务数据作为首个文字项插入已构建的内容数组，再整体封装发布。派生类可增加输入端口并重写 `buildPayload()`，例如把传入的错误码与动作名组合成一条导航失败消息。所有 Event payload 均在 `buildPayload()` 中构建。

## 5. 对接现有 EventManager

沿用现有话题和类型：`/vln/robot_event`，`std_msgs/msg/String`。QoS 与接收端保持一致：reliable、volatile、KeepLast(10)。

String 的内容按现有约定包装，不直接发送裸业务对象：

```cpp
auto event = buildPayload();
Json text = {{"type", "input_text"}, {"text", event.business.dump()}};
event.image_content.insert(event.image_content.begin(), text);
Json message = {{"role", "user"}, {"content", std::move(event.image_content)}};
// 将 message.dump() 放入 std_msgs::msg::String::data 后发布。
```

这部分直接复用现有接收协议，不需要修改 `vln_agent`。

## 6. 行为树中的使用

事件字段可以直接通过输入端口配置：

```xml
<SubmitEvent name="ReportNavigationFailure"
             event="error"
             text="导航执行失败。"/>
```

需要动作状态、传感器等业务数据时，通过黑板连接相应输入端口，由 `buildPayload()` 构建 payload。这个节点可以放在恢复、失败、成功或其他需要上报的分支中；分支选择与后续行为由树中其他节点决定。

后续分别在以下文件选择上报位置：

- `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- `behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`

当前先实现通用节点，具体位置在调整行为树时确定。

## 7. 第一部分的实现范围

主要文件保持为：

```text
include/vln_behavior_tree/image_content.hpp
include/vln_behavior_tree/event_node_base.hpp
src/event_node_base.cpp
```

在实现文件中注册 `SubmitEvent`，并在本包构建文件中补充依赖、编译与安装规则。需要专用节点时，再增加对应的派生类文件。

验证只围绕实际功能：`buildPayload()` 能根据输入数据构建 payload、填写公共字段，生成的事件能正确发布、拼接图片内容，以及派生类能按需组织输入。使用普通 ROS 测试订阅者即可，无需连接模型或改动 Agent。

## 8. 实现记录

2026-10-31：已实现 `EventNodeBase` 并注册为 `SubmitEvent`，提供可导出的基础库、头文件和动态插件。开发机 Humble / BehaviorTree.CPP 3.8.7 下 Release 编译、6 项功能测试和 4 项静态检查通过；安装后独立消费者的派生类链接与动态插件加载验证通过。

2026-10-31 图片输入更新：新增 `ImageContent`，由图片文件字节、任意键名元数据 map 和媒体类型组成。`buildPayload()` 返回 `EventPayload`，统一完成 Base64 编码和完整元数据转换；时间与编码复用 `vln_agent_utils`。开发机 7 项功能测试、4 项静态检查及安装后独立消费者验证通过。
