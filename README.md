# vln_behavior_tree

面向 ROS 2 Humble / Nav2 的视觉语言导航行为树插件。项目提供事件上报、Agent 工具调用、彩色图像获取、环视图提交和导航目标表达式更新节点，并提供单目标、多目标与环顾行为树。

行为树通过 ROS 话题将观测和导航事件交给 `vln_agent`，也可通过工具服务复用 Agent 的执行资源。节点端口、消息格式和导航流程见 [节点与导航流程文档](docs/BEHAVIOR_TREE_PLAN.md)。

## 环境与依赖

- Ubuntu 22.04、ROS 2 Humble、Nav2、BehaviorTree.CPP 3。
- 同工作空间中的 `vln_agent` 和 `rgbd_image_queue`。
- TF2、OpenCV 及相关 ROS 消息包。

将相关包放入同一 ROS 工作空间，通过工作空间的依赖管理安装所需 ROS 和系统库。

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vln_behavior_tree --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

资源较紧张的设备可以在构建命令前设置 `CMAKE_BUILD_PARALLEL_LEVEL=1`。目标表达式解析器随包提供。

## 测试

```bash
colcon test --packages-select vln_behavior_tree --event-handlers console_direct+
colcon test-result --test-result-base build/vln_behavior_tree --verbose
```

测试使用本地 ROS 服务、合成图像和导航替身，覆盖事件发布、工具调用、图片集合、环视合成、目标更新和导航树流程。

## 使用

编译后将所需插件加入 Nav2 的 `bt_navigator.ros__parameters.plugin_lib_names`，并选择包安装目录中的行为树。默认接入话题和服务由 Agent 及节点端口配置。

项目中的三个行为树：

- `behavior_trees/vln_nav2goal.xml`：单目标导航。
- `behavior_trees/vln_navthroughgoals.xml`：多目标导航。
- `behavior_trees/lookaround.xml`：八次取图和转向，完成后提交环视图。

完整插件列表、Nav2 接入方式与 XML 示例见 [节点与导航流程文档](docs/BEHAVIOR_TREE_PLAN.md)。

## 项目文档

- [节点、数据类型与导航流程](docs/BEHAVIOR_TREE_PLAN.md)
- [行为树 XML](behavior_trees)
- [公共头文件](include/vln_behavior_tree)
- [测试源码](test)
