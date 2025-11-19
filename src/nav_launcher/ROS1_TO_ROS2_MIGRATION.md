# ROS1 到 ROS2 Nav2 迁移指南

## 🔄 主要变化对比

### 架构变化

| 方面 | ROS1 (Navigation Stack) | ROS2 (Nav2) |
|------|------------------------|-------------|
| **包名** | navigation | nav2_* (多个独立包) |
| **启动文件** | XML (.launch) | Python (.launch.py) |
| **参数管理** | 全局参数服务器 | 节点本地参数 + YAML |
| **节点通信** | Topics/Services/Actions | 相同，但使用DDS |
| **生命周期** | 无标准生命周期 | 托管节点生命周期 |
| **行为树** | 不支持 | 原生支持 |
| **组合节点** | 不支持 | 支持（提升性能）|

### 包结构对比

**ROS1:**
```
my_robot_navigation/
├── launch/
│   └── navigation.launch        # XML 格式
├── config/
│   ├── costmap_common.yaml
│   ├── global_costmap.yaml
│   ├── local_costmap.yaml
│   └── base_local_planner.yaml
└── maps/
```

**ROS2:**
```
my_robot_navigation/
├── launch/
│   └── navigation.launch.py     # Python 格式
├── params/
│   └── nav2_params.yaml         # 统一配置文件
├── maps/
├── rviz/
└── README.md
```

---

## 📝 配置文件迁移

### 1. 启动文件迁移

**ROS1 (.launch):**
```xml
<launch>
  <node pkg="move_base" type="move_base" name="move_base">
    <rosparam file="$(find my_nav)/config/costmap_common.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find my_nav)/config/costmap_common.yaml" command="load" ns="local_costmap"/>
    <rosparam file="$(find my_nav)/config/local_costmap.yaml" command="load"/>
    <rosparam file="$(find my_nav)/config/global_costmap.yaml" command="load"/>
    <rosparam file="$(find my_nav)/config/base_local_planner.yaml" command="load"/>
  </node>
</launch>
```

**ROS2 (.launch.py):**
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(get_package_share_directory('my_nav'), 
                               'params', 'nav2_params.yaml')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'params_file': params_file}.items()
        )
    ])
```

### 2. 参数配置迁移

**ROS1 参数结构:**
```yaml
# costmap_common.yaml
footprint: [[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]
obstacle_range: 2.5
raytrace_range: 3.0

# local_costmap.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  
# base_local_planner.yaml
TrajectoryPlannerROS:
  max_vel_x: 0.5
  max_vel_theta: 1.0
```

**ROS2 统一参数文件:**
```yaml
# nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          obstacle_range: 2.5
          raytrace_range: 3.0

controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0
```

---

## 🔧 常用命令对比

| 功能 | ROS1 | ROS2 |
|------|------|------|
| **启动节点** | `roslaunch` | `ros2 launch` |
| **查看话题** | `rostopic list` | `ros2 topic list` |
| **发布话题** | `rostopic pub` | `ros2 topic pub` |
| **查看参数** | `rosparam list` | `ros2 param list` |
| **设置参数** | `rosparam set` | `ros2 param set` |
| **查看TF** | `rosrun tf view_frames` | `ros2 run tf2_tools view_frames` |
| **编译** | `catkin_make` | `colcon build` |
| **Source** | `source devel/setup.bash` | `source install/setup.bash` |

### 详细命令示例

**ROS1:**
```bash
# 启动导航
roslaunch my_robot_navigation navigation.launch

# 设置目标点
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...

# 取消目标
rostopic pub /move_base/cancel actionlib_msgs/GoalID ...

# 查看参数
rosparam get /move_base/global_costmap/width
```

**ROS2:**
```bash
# 启动导航
ros2 launch nav_launcher unitree_navigation.launch.py

# 设置目标点
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...

# 取消目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose --feedback

# 查看参数
ros2 param get /global_costmap/global_costmap width
```

---

## 📦 节点名称映射

| ROS1 | ROS2 | 说明 |
|------|------|------|
| `move_base` | `controller_server` + `planner_server` + `bt_navigator` | 分离为多个节点 |
| `amcl` | `amcl` | 基本保持一致 |
| `map_server` | `map_server` | 基本保持一致 |
| - | `behavior_server` | 新增恢复行为 |
| - | `waypoint_follower` | 新增路点跟随 |
| - | `lifecycle_manager` | 新增生命周期管理 |

---

## 🆕 ROS2 Nav2 新特性

### 1. 行为树 (Behavior Trees)
ROS2 Nav2 使用行为树替代 ROS1 的状态机。

**优势:**
- 更灵活的导航逻辑
- 可视化编辑（Groot）
- 易于扩展和调试

**使用:**
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
```

### 2. 生命周期节点
所有 Nav2 节点都是托管生命周期节点。

**状态:**
- Unconfigured（未配置）
- Inactive（非活动）
- Active（活动）
- Finalized（终结）

**命令:**
```bash
# 查看节点状态
ros2 lifecycle get /controller_server

# 改变状态
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```

### 3. 组合节点
将多个节点组合到单个进程，减少通信开销。

**启用:**
```bash
ros2 launch nav_launcher unitree_navigation.launch.py use_composition:=True
```

### 4. 更多规划器选项

| 规划器 | 特点 | 适用场景 |
|--------|------|---------|
| NavFn | Dijkstra | 简单环境 |
| SmacPlanner2D | A* | 栅格地图 |
| SmacPlannerHybrid | Hybrid A* | 类车机器人 |
| ThetaStar | Any-angle | 需要平滑路径 |

### 5. 更多控制器选项

| 控制器 | 特点 | 适用场景 |
|--------|------|---------|
| DWB | 动态窗口法 | 差速机器人 |
| TEB | 时间弹性带 | 复杂环境 |
| MPPI | 模型预测 | 高级应用 |
| RPP | 纯跟踪 | 简单快速 |

---

## ⚠️ 常见迁移陷阱

### 1. 话题名称空间
ROS2 更严格的命名空间规则。

**问题:**
```yaml
# 这在 ROS2 中可能不工作
scan_topic: /scan  # 绝对路径
```

**解决:**
```yaml
# 使用相对路径
scan_topic: scan   # 相对路径（推荐）
```

### 2. 坐标系变换
确保 TF 树完整。

**ROS1:** 有些驱动会自动发布某些TF
**ROS2:** 需要显式配置所有TF

### 3. 参数嵌套
ROS2 需要 `ros__parameters` 关键字。

**错误:**
```yaml
controller_server:
  controller_frequency: 20.0  # 错误！
```

**正确:**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # 正确
```

### 4. Action 接口变化
ROS2 的 Action 接口有所不同。

**ROS1:**
```
/move_base/goal
/move_base/result
/move_base/feedback
```

**ROS2:**
```
/navigate_to_pose/_action/send_goal
/navigate_to_pose/_action/get_result
/navigate_to_pose/_action/feedback
```

---

## 🎯 迁移步骤建议

### 阶段 1: 准备（1-2天）
1. [ ] 学习 ROS2 基础概念
2. [ ] 安装 ROS2 和 Nav2
3. [ ] 熟悉新的命令行工具
4. [ ] 创建新的工作空间

### 阶段 2: 基础迁移（3-5天）
1. [ ] 创建新的 ROS2 包
2. [ ] 迁移启动文件（XML -> Python）
3. [ ] 合并配置文件（多个 YAML -> 单个 YAML）
4. [ ] 更新坐标系和话题名称

### 阶段 3: 测试和调试（5-7天）
1. [ ] 在仿真环境测试
2. [ ] 调整参数
3. [ ] 验证 TF 树
4. [ ] 测试定位和导航

### 阶段 4: 优化（3-5天）
1. [ ] 性能调优
2. [ ] 启用高级功能
3. [ ] 在实际机器人上测试
4. [ ] 编写文档

---

## 📚 学习资源

### 官方文档
- [Nav2 官方文档](https://docs.nav2.org/)
- [ROS2 官方教程](https://docs.ros.org/en/humble/Tutorials.html)
- [Nav2 迁移指南](https://docs.nav2.org/migration/Galactic.html)

### 视频教程
- [Nav2 简介系列](https://www.youtube.com/c/OpenRobotics)
- [ROS2 Navigation 实战](https://www.youtube.com/watch?v=...)

### 社区资源
- [Nav2 GitHub](https://github.com/ros-planning/navigation2)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

---

## 💡 最佳实践

1. **使用组合节点** - 在资源受限的平台上提升性能
2. **保持参数文件整洁** - 使用注释说明每个参数
3. **版本控制** - 将配置文件纳入 git 管理
4. **增量迁移** - 先迁移基础功能，再添加高级特性
5. **充分测试** - 在仿真环境充分测试后再部署到真机
6. **记录问题** - 记录遇到的问题和解决方案

---

希望这份迁移指南能帮助你顺利从 ROS1 过渡到 ROS2！🚀
