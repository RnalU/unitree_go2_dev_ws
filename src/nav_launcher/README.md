# Nav2 配置和使用指南

## 📚 目录结构
```
nav_launcher/
├── launch/              # 启动文件
│   ├── unitree_navigation.launch.py         # 完整导航启动
│   └── localization_and_navigation.launch.py # 简化版启动
├── params/              # 参数配置
│   └── unitree_nav2_params.yaml              # Nav2参数配置
├── maps/                # 地图文件
│   └── my_map.yaml                           # 你的地图
├── rviz/                # RVIZ配置
└── README.md            # 本文件
```

## 🚀 快速开始

### 1. 构建工作空间
```bash
cd /home/ymc/ros2_nav_unitree_ws
colcon build --packages-select nav_launcher
source install/setup.zsh
```

### 2. 启动导航系统
```bash
# 方式1：完整启动（推荐）
ros2 launch nav_launcher unitree_navigation.launch.py

# 方式2：简化启动
ros2 launch nav_launcher localization_and_navigation.launch.py

# 方式3：指定地图和参数
ros2 launch nav_launcher unitree_navigation.launch.py \
    map:=/path/to/your/map.yaml \
    params_file:=/path/to/your/params.yaml
```

### 3. 在RVIZ中设置初始位姿
1. 点击 "2D Pose Estimate" 按钮
2. 在地图上点击并拖动设置机器人位置和方向

### 4. 发送导航目标
1. 点击 "Nav2 Goal" 按钮
2. 在地图上点击目标位置

## ⚙️ ROS2 vs ROS1 主要区别

### 启动文件
- **ROS1**: XML格式 (.launch)
- **ROS2**: Python格式 (.launch.py)，更灵活强大

### 参数服务器
- **ROS1**: 全局参数服务器 (rosparam)
- **ROS2**: 每个节点独立参数，通过YAML文件配置

### 话题通信
- **ROS1**: rostopic
- **ROS2**: ros2 topic

### 包管理
- **ROS1**: catkin_make / catkin build
- **ROS2**: colcon build

## 📝 重要配置项说明

### 1. 坐标系配置
需要确保你的机器人发布以下TF变换：
```
map -> odom -> base_link -> laser_frame
```

- `map`: 全局地图坐标系
- `odom`: 里程计坐标系（由你的机器人发布）
- `base_link`: 机器人基座坐标系
- `laser_frame`: 激光雷达坐标系

### 2. 话题名称
在 `unitree_nav2_params.yaml` 中需要修改的话题：
- `scan_topic`: 激光雷达话题（默认 `/scan`）
- `odom_topic`: 里程计话题（默认 `/odom`）

### 3. 机器人参数（重要！）
根据你的Unitree机器人修改：
```yaml
robot_radius: 0.22           # 机器人半径（米）
max_vel_x: 0.5              # 最大线速度（米/秒）
max_vel_theta: 1.0          # 最大角速度（弧度/秒）
acc_lim_x: 2.5              # 线加速度限制
acc_lim_theta: 3.2          # 角加速度限制
```

### 4. 传感器配置
激光雷达配置（在costmap层中）：
```yaml
observation_sources: scan
scan:
  topic: /scan              # 话题名称
  max_obstacle_height: 2.0  # 最大障碍物高度
  obstacle_max_range: 2.5   # 障碍物最大范围
  raytrace_max_range: 3.0   # 光线追踪最大范围
```

### 5. 规划器选择

#### 全局规划器（Planner Server）
- **NavFn Planner**: Dijkstra算法，简单可靠
- **SmacPlanner2D**: A*算法，适合大地图
- **SmacPlannerHybrid**: 混合A*，适合车辆式机器人

#### 局部控制器（Controller Server）
- **DWB Controller**: 动态窗口法，适合差速机器人（推荐）
- **TEB Controller**: 时间弹性带，适合复杂环境
- **MPPI Controller**: 模型预测控制，高级应用
- **RPP Controller**: 纯跟踪控制器，简单快速

当前配置使用 DWB Controller，适合大多数差速机器人。

## 🗺️ 地图准备

### 方式1：使用已有地图
将地图文件放到 `maps/` 目录：
- `my_map.pgm`: 地图图像
- `my_map.yaml`: 地图元数据

### 方式2：使用SLAM建图
```bash
# 启动SLAM建图
ros2 launch nav_launcher unitree_navigation.launch.py slam:=True

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/my_new_map
```

## 🔧 调试技巧

### 查看话题
```bash
# 查看所有话题
ros2 topic list

# 查看激光数据
ros2 topic echo /scan

# 查看里程计
ros2 topic echo /odom

# 查看TF树
ros2 run tf2_tools view_frames
```

### 查看节点
```bash
# 查看所有节点
ros2 node list

# 查看节点信息
ros2 node info /controller_server
```

### 查看参数
```bash
# 查看节点参数
ros2 param list /controller_server

# 获取参数值
ros2 param get /controller_server controller_frequency
```

### 实时修改参数
```bash
# 修改最大速度
ros2 param set /controller_server FollowPath.max_vel_x 0.3
```

## 📊 性能优化

### 1. 使用组合节点（Composition）
已在启动文件中默认启用 `use_composition:=True`
可以将多个节点组合到一个进程，减少通信开销。

### 2. 调整更新频率
```yaml
controller_frequency: 20.0    # 控制器频率（Hz）
costmap_update_frequency: 5.0 # 代价地图更新频率
```

### 3. 调整地图大小
```yaml
local_costmap:
  width: 3   # 局部地图宽度（米）
  height: 3  # 局部地图高度（米）
```

## ⚠️ 常见问题

### 1. 机器人不动
- 检查 `/cmd_vel` 话题是否有数据：`ros2 topic echo /cmd_vel`
- 确认机器人控制器订阅了 `/cmd_vel`
- 检查速度限制是否太小

### 2. 定位不准确
- 调整AMCL粒子数：`max_particles`, `min_particles`
- 检查激光数据：`ros2 topic echo /scan`
- 确认TF树正确发布

### 3. 路径规划失败
- 检查机器人半径是否设置正确
- 查看代价地图：在RVIZ中添加 `/global_costmap/costmap`
- 增加规划器容差：`tolerance`

### 4. 碰撞检测过于敏感
- 调整膨胀半径：`inflation_radius`
- 调整代价缩放因子：`cost_scaling_factor`

## 🔗 有用的命令

```bash
# 编译单个包
colcon build --packages-select nav_launcher

# 仅编译并运行测试
colcon test --packages-select nav_launcher

# 清理构建
rm -rf build install log

# 查看构建日志
colcon build --event-handlers console_direct+

# 设置Nav2目标（命令行）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 📖 参考资源

- [Nav2官方文档](https://docs.nav2.org/)
- [Nav2配置指南](https://docs.nav2.org/configuration/index.html)
- [Nav2调优指南](https://docs.nav2.org/tuning/index.html)
- [ROS2入门教程](https://docs.ros.org/en/humble/Tutorials.html)

## 🎯 下一步

1. **配置机器人参数**：根据Unitree机器人规格修改 `unitree_nav2_params.yaml`
2. **准备地图**：使用SLAM建图或导入已有地图到 `maps/` 目录
3. **测试传感器**：确认激光雷达和里程计数据正常
4. **启动导航**：运行启动文件并测试
5. **参数调优**：根据实际表现调整速度、加速度等参数

## 💡 提示

- 创建独立的导航包是**正确的做法**，与ROS1习惯一致
- Python启动文件自动生成是正常的，这是ROS2的标准方式
- 建议先在仿真环境测试，再部署到实际机器人
- 保持参数文件版本控制，便于回滚和对比

祝你使用愉快！🎉
