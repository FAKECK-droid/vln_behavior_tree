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

1. 调用 `buildPayload()`，根据输入数据构建完整业务 payload，包括公共字段。
2. 将 payload 包装为现有 EventManager 接收的消息格式。
3. 发布到配置的话题，正常返回 `SUCCESS`；输入解析或发布发生异常时记录日志并返回 `FAILURE`。

`SUCCESS` 表示本次本地发布调用完成。节点不等待接收回执或模型处理结果。

重复上报、上报频率以及失败后是否重试，由行为树组合其他节点决定，Event 节点不额外维护这些策略。

## 3. 输入端口

| 端口 | 类型 | 用途 |
| --- | --- | --- |
| `topic` | string | 上报话题，默认 `/vln/robot_event` |
| `event` | string | 事件类型，默认 `status` |
| `text` | string | 事件说明文字 |
| `image_content` | JSON 数组，可选 | 已准备好的原生图片内容项；没有图片时为空 |

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

图片内容通过 `image_content` 输入，每项可附带 `camera`、`time_stamp`，由 `buildPayload()` 按顺序填写到 `payload.images`。消息包装时，图片项中的采样元数据移入业务字段，原生图片内容追加到 `content`。

## 4. 基类接口草案

```cpp
class EventNodeBase : public BT::SyncActionNode
{
public:
  EventNodeBase(const std::string & name,
                const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

protected:
  // 根据输入数据构建完整的 Event 业务 payload。
  virtual Json buildPayload();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_;
};
```

构造时从 Nav2 黑板的 `node` 获取宿主 ROS 节点，发布器按 `topic` 创建并复用。JSON 使用本包直接声明的库依赖。

基类的 `buildPayload()` 根据公共输入端口构建 payload，`tick()` 负责调用该函数、包装消息并发布。派生类可增加输入端口并重写 `buildPayload()`，例如把传入的错误码与动作名组合成一条导航失败消息。所有 Event payload 均在 `buildPayload()` 中构建。

## 5. 对接现有 EventManager

沿用现有话题和类型：`/vln/robot_event`，`std_msgs/msg/String`。QoS 与接收端保持一致：reliable、volatile、KeepLast(10)。

String 的内容按现有约定包装，不直接发送裸业务对象：

```cpp
Json content = Json::array({
  {{"type", "input_text"}, {"text", payload.dump()}}
});
for (const auto & image : image_content) {
  content.push_back(image);
}
Json message = {{"role", "user"}, {"content", content}};
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
include/vln_behavior_tree/event_node_base.hpp
src/event_node_base.cpp
```

在实现文件中注册 `SubmitEvent`，并在本包构建文件中补充依赖、编译与安装规则。需要专用节点时，再增加对应的派生类文件。

验证只围绕实际功能：`buildPayload()` 能根据输入数据构建 payload、填写公共字段，生成的事件能正确发布、拼接图片内容，以及派生类能按需组织输入。使用普通 ROS 测试订阅者即可，无需连接模型或改动 Agent。

## 8. 实现记录

2026-10-31：已实现 `EventNodeBase` 并注册为 `SubmitEvent`，提供可导出的基础库、头文件和动态插件。开发机 Humble / BehaviorTree.CPP 3.8.7 下 Release 编译、6 项功能测试和 4 项静态检查通过；安装后独立消费者的派生类链接与动态插件加载验证通过。
