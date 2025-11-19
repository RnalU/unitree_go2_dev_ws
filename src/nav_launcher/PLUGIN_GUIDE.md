# 🔌 Nav2 Plugin 查找和使用指南

## 什么是 Plugin？

在 Nav2 中，**Plugin** 是可插拔的功能模块。你可以轻松切换不同的算法实现，而不需要修改核心代码。

### Plugin 命名规则

```
package_name::ClassName
    ↑            ↑
 包名/命名空间   类名
```

**示例：**
- `dwb_core::DWBLocalPlanner` 
  - 包名：`dwb_core`
  - 类名：`DWBLocalPlanner`
  
- `nav2_teb_controller::TebController`
  - 包名：`nav2_teb_controller`
  - 类名：`TebController`

---

## 🔍 如何查找 Plugin 名称

### 方法 1: 查看 Nav2 官方文档（推荐）

访问：https://docs.nav2.org/plugins/index.html

**官方插件列表：**

#### 控制器（Controllers）
| Plugin 名称 | 说明 | 适用场景 |
|------------|------|---------|
| `dwb_core::DWBLocalPlanner` | 动态窗口法 | 差速机器人 |
| `nav2_teb_controller::TebController` | 时间弹性带 | 复杂环境、四足机器人 |
| `nav2_mppi_controller::MPPIController` | 模型预测控制 | 高级应用 |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 纯跟踪 | 类车机器人 |
| `nav2_graceful_controller::GracefulController` | 优雅控制 | 平滑运动 |
| `nav2_rotation_shim_controller::RotationShimController` | 旋转垫片 | 大角度调整 |

#### 规划器（Planners）
| Plugin 名称 | 说明 | 适用场景 |
|------------|------|---------|
| `nav2_navfn_planner/NavfnPlanner` | Dijkstra/A* | 栅格地图 |
| `nav2_smac_planner/SmacPlanner2D` | A* 2D | 大地图 |
| `nav2_smac_planner/SmacPlannerHybrid` | Hybrid A* | 类车机器人 |
| `nav2_smac_planner/SmacPlannerLattice` | Lattice | 复杂约束 |
| `nav2_theta_star_planner/ThetaStarPlanner` | Theta* | 任意角度 |

#### 平滑器（Smoothers）
| Plugin 名称 | 说明 |
|------------|------|
| `nav2_smoother::SimpleSmoother` | 简单平滑 |
| `nav2_constrained_smoother::ConstrainedSmoother` | 约束平滑 |

#### 目标检查器（Goal Checkers）
| Plugin 名称 | 说明 |
|------------|------|
| `nav2_controller::SimpleGoalChecker` | 简单目标检查 |
| `nav2_controller::StoppedGoalChecker` | 停止目标检查 |

#### 进度检查器（Progress Checkers）
| Plugin 名称 | 说明 |
|------------|------|
| `nav2_controller::SimpleProgressChecker` | 简单进度检查 |
| `nav2_controller::PoseProgressChecker` | 位姿进度检查 |

---

### 方法 2: 查看包的 plugins.xml 文件

每个提供 Plugin 的包都有一个 `plugins.xml` 文件。

**示例：查看 TEB 控制器的 Plugin 名称**

```bash
# 1. 找到包的路径
ros2 pkg prefix nav2_teb_controller

# 输出：/opt/ros/humble/share/nav2_teb_controller

# 2. 查看 plugins.xml
cat /opt/ros/humble/share/nav2_teb_controller/plugins.xml
```

**plugins.xml 内容：**
```xml
<class_libraries>
  <library path="nav2_teb_controller">
    <class type="nav2_teb_controller::TebController" 
           base_class_type="nav2_core::Controller">
      <description>
        TEB local controller plugin
      </description>
    </class>
  </library>
</class_libraries>
```

从这里可以看到：
- **Plugin 名称**：`nav2_teb_controller::TebController`
- **基类**：`nav2_core::Controller`
- **描述**：TEB local controller plugin

---

### 方法 3: 查看源码

```bash
# 进入 Nav2 源码目录
cd /home/ymc/ros2_nav_unitree_ws/src/navigation2

# 查找所有 plugins.xml 文件
find . -name "plugins.xml"

# 输出：
# ./nav2_dwb_controller/plugins.xml
# ./nav2_teb_controller/plugins.xml
# ./nav2_mppi_controller/plugins.xml
# ...
```

**查看具体文件：**
```bash
cat ./nav2_dwb_controller/plugins.xml
```

---

### 方法 4: 使用 ros2 命令（如果已安装）

```bash
# 列出所有控制器插件
ros2 plugin list nav2_core::Controller

# 列出所有规划器插件
ros2 plugin list nav2_core::GlobalPlanner

# 列出所有平滑器插件
ros2 plugin list nav2_core::Smoother
```

---

## 📦 Nav2 完整 Plugin 列表

### 1. 控制器 (Controller Plugins)

```yaml
# DWB - 动态窗口法（推荐用于差速机器人）
plugin: "dwb_core::DWBLocalPlanner"

# TEB - 时间弹性带（推荐用于四足机器人、复杂环境）
plugin: "nav2_teb_controller::TebController"

# MPPI - 模型预测路径积分（高级）
plugin: "nav2_mppi_controller::MPPIController"

# RPP - 调节纯跟踪（适合类车机器人）
plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

# Graceful - 优雅控制器
plugin: "nav2_graceful_controller::GracefulController"

# Rotation Shim - 旋转垫片（大角度转向）
plugin: "nav2_rotation_shim_controller::RotationShimController"
```

### 2. 规划器 (Planner Plugins)

```yaml
# NavFn - Dijkstra/A*（默认）
plugin: "nav2_navfn_planner/NavfnPlanner"

# Smac 2D - A* 2D
plugin: "nav2_smac_planner/SmacPlanner2D"

# Smac Hybrid - Hybrid A*（类车机器人）
plugin: "nav2_smac_planner/SmacPlannerHybrid"

# Smac Lattice - Lattice 规划器
plugin: "nav2_smac_planner/SmacPlannerLattice"

# Theta Star - Theta*（任意角度）
plugin: "nav2_theta_star_planner/ThetaStarPlanner"
```

### 3. 平滑器 (Smoother Plugins)

```yaml
# Simple Smoother - 简单平滑
plugin: "nav2_smoother::SimpleSmoother"

# Constrained Smoother - 约束平滑
plugin: "nav2_constrained_smoother::ConstrainedSmoother"
```

### 4. 代价地图层 (Costmap Layer Plugins)

```yaml
# Static Layer - 静态地图层
plugin: "nav2_costmap_2d::StaticLayer"

# Obstacle Layer - 障碍物层
plugin: "nav2_costmap_2d::ObstacleLayer"

# Voxel Layer - 体素层（3D）
plugin: "nav2_costmap_2d::VoxelLayer"

# Inflation Layer - 膨胀层
plugin: "nav2_costmap_2d::InflationLayer"

# Range Sensor Layer - 距离传感器层
plugin: "nav2_costmap_2d::RangeSensorLayer"

# Denoise Layer - 降噪层
plugin: "nav2_costmap_2d::DenoiseLayer"
```

### 5. 行为插件 (Behavior Plugins)

```yaml
# Spin - 原地旋转
plugin: "nav2_behaviors::Spin"

# BackUp - 后退
plugin: "nav2_behaviors::BackUp"

# DriveOnHeading - 朝向行驶
plugin: "nav2_behaviors::DriveOnHeading"

# Wait - 等待
plugin: "nav2_behaviors::Wait"

# AssistedTeleop - 辅助遥控
plugin: "nav2_behaviors::AssistedTeleop"
```

### 6. 路点任务执行器 (Waypoint Task Executor Plugins)

```yaml
# Wait at Waypoint - 在路点等待
plugin: "nav2_waypoint_follower::WaitAtWaypoint"

# Photo at Waypoint - 在路点拍照
plugin: "nav2_waypoint_follower::PhotoAtWaypoint"

# Input at Waypoint - 在路点输入
plugin: "nav2_waypoint_follower::InputAtWaypoint"
```

---

## 🔄 如何切换 Plugin

### 示例：从 DWB 切换到 TEB

**修改前（DWB）：**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      # ... DWB 参数 ...
```

**修改后（TEB）：**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_teb_controller::TebController"
      max_vel_x: 0.5
      # ... TEB 参数 ...
```

**关键点：**
1. ✅ 只需修改 `plugin` 字段
2. ✅ 参数需要根据新 Plugin 调整
3. ✅ `FollowPath` 是插件实例的名称，可以自定义
4. ✅ 可以同时使用多个插件

---

## 🎯 多插件配置

### 示例 1: 多个控制器

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["DWBController", "TEBController", "RPPController"]
    
    DWBController:
      plugin: "dwb_core::DWBLocalPlanner"
      # DWB 参数...
    
    TEBController:
      plugin: "nav2_teb_controller::TebController"
      # TEB 参数...
    
    RPPController:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # RPP 参数...
```

**运行时切换：**
```bash
# 使用 DWB
ros2 param set /controller_server controller_plugins '["DWBController"]'

# 使用 TEB
ros2 param set /controller_server controller_plugins '["TEBController"]'
```

### 示例 2: 多个规划器

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["NavFn", "SmacHybrid"]
    
    NavFn:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
    
    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
```

**在代码中指定：**
```python
# Python
goal_msg = NavigateToPose.Goal()
goal_msg.planner_id = "SmacHybrid"  # 使用 Smac 规划器
```

---

## 🧪 测试不同 Plugin

### 快速测试脚本

创建文件 `test_plugins.sh`：

```bash
#!/bin/bash

echo "测试不同的控制器插件"

# 测试 DWB
echo "1. 测试 DWB 控制器..."
ros2 param set /controller_server FollowPath.plugin "dwb_core::DWBLocalPlanner"
sleep 2

# 测试 TEB
echo "2. 测试 TEB 控制器..."
ros2 param set /controller_server FollowPath.plugin "nav2_teb_controller::TebController"
sleep 2

# 测试 MPPI
echo "3. 测试 MPPI 控制器..."
ros2 param set /controller_server FollowPath.plugin "nav2_mppi_controller::MPPIController"
sleep 2

echo "测试完成！"
```

---

## 📚 Plugin 开发（高级）

### 创建自定义 Plugin

如果官方 Plugin 不满足需求，可以开发自定义 Plugin。

**基本步骤：**

1. **创建插件类**
```cpp
// my_controller.hpp
#include "nav2_core/controller.hpp"

namespace my_nav_plugins {

class MyController : public nav2_core::Controller {
public:
  void configure(/* ... */) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(/* ... */) override;
};

}  // namespace my_nav_plugins
```

2. **注册插件**
```cpp
// my_controller.cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_nav_plugins::MyController, nav2_core::Controller)
```

3. **创建 plugins.xml**
```xml
<library path="my_nav_plugins">
  <class type="my_nav_plugins::MyController" base_class_type="nav2_core::Controller">
    <description>My custom controller</description>
  </class>
</library>
```

4. **在 package.xml 中声明**
```xml
<export>
  <nav2_core plugin="${prefix}/plugins.xml"/>
</export>
```

5. **使用自定义插件**
```yaml
controller_plugins: ["MyController"]
MyController:
  plugin: "my_nav_plugins::MyController"
```

---

## 🔍 故障排查

### 问题 1: Plugin 加载失败

**错误信息：**
```
Failed to load plugin: nav2_teb_controller::TebController
```

**检查清单：**
```bash
# 1. 检查包是否安装
ros2 pkg list | grep nav2_teb_controller

# 2. 检查 plugins.xml 是否存在
ls $(ros2 pkg prefix nav2_teb_controller)/share/nav2_teb_controller/plugins.xml

# 3. 检查拼写是否正确
# plugin: "nav2_teb_controller::TebController" ← 正确
# plugin: "nav2_teb_controller::TEBController" ← 错误（大小写）

# 4. 重新构建工作空间
cd /home/ymc/ros2_nav_unitree_ws
colcon build --packages-select nav2_teb_controller
source install/setup.zsh
```

### 问题 2: 参数不兼容

不同 Plugin 的参数不同，切换时需要更新参数。

**解决方案：**
- 为每个 Plugin 准备单独的参数文件
- 或使用条件加载参数

---

## 💡 最佳实践

1. **先使用官方 Plugin**
   - 官方 Plugin 经过充分测试
   - 文档完善，社区支持好

2. **参数文件管理**
   ```
   params/
   ├── unitree_nav2_dwb.yaml      # DWB 配置
   ├── unitree_nav2_teb.yaml      # TEB 配置
   └── unitree_nav2_mppi.yaml     # MPPI 配置
   ```

3. **记录测试结果**
   - 不同 Plugin 的性能对比
   - 适用场景记录

4. **版本兼容性**
   - 检查 Plugin 是否支持你的 ROS2 版本
   - 查看 package.xml 中的依赖

---

## 📖 相关资源

- [Nav2 Plugins 文档](https://docs.nav2.org/plugins/index.html)
- [Pluginlib 教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
- [Nav2 源码](https://github.com/ros-planning/navigation2)

---

**总结：**

- 🔌 Plugin 是 Nav2 的核心设计，提供灵活性
- 📝 Plugin 命名格式：`package_name::ClassName`
- 🔍 多种方法查找 Plugin 名称
- 🔄 轻松切换不同算法实现
- 🎯 可同时配置多个 Plugin

希望这份指南帮助你掌握 Nav2 Plugin 系统！🚀
