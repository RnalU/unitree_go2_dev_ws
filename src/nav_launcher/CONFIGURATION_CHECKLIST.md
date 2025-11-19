# 🚀 ROS2 Nav2 配置清单 - Unitree 机器人

这是一份详细的配置清单，帮助你逐步配置和调试Nav2导航系统。

---

## ✅ 准备工作清单

### 1. 机器人硬件信息收集
在开始配置之前，你需要知道以下信息：

- [ ] **机器人类型**：差速 / 全向 / 阿克曼
- [ ] **机器人尺寸**：
  - 长度：_____ 米
  - 宽度：_____ 米  
  - 高度：_____ 米
  - 安全半径：_____ 米（建议值 = 对角线长度/2 + 0.05）
  
- [ ] **运动参数**：
  - 最大线速度：_____ 米/秒
  - 最大角速度：_____ 弧度/秒
  - 最大加速度：_____ 米/秒²
  - 最大角加速度：_____ 弧度/秒²

- [ ] **传感器配置**：
  - 激光雷达型号：_____
  - 激光雷达话题：_____ (例如：/scan)
  - 激光雷达最大范围：_____ 米
  - 是否有深度相机：是 / 否
  - 其他传感器：_____

- [ ] **坐标系名称**：
  - 地图坐标系：_____ (默认: map)
  - 里程计坐标系：_____ (默认: odom)
  - 机器人基座：_____ (默认: base_link)
  - 激光坐标系：_____ (例如: laser_frame)

---

## 📋 配置步骤

### 步骤 1: 验证基础话题 ✓

运行以下命令验证必要的话题是否存在：

```bash
# 检查激光雷达数据
ros2 topic echo /scan --once

# 检查里程计数据  
ros2 topic echo /odom --once

# 列出所有话题
ros2 topic list
```

**预期结果**：
- [ ] `/scan` 话题正常发布 (sensor_msgs/LaserScan)
- [ ] `/odom` 话题正常发布 (nav_msgs/Odometry)
- [ ] 数据更新频率 > 5Hz

---

### 步骤 2: 验证 TF 变换树 ✓

TF树是Nav2工作的基础！

```bash
# 安装tf2工具（如果还没有）
sudo apt install ros-humble-tf2-tools

# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

**预期结果**：
- [ ] TF树完整：map -> odom -> base_link -> laser_frame
- [ ] odom -> base_link 变换正常更新（由机器人驱动发布）
- [ ] base_link -> laser_frame 变换正常（通常由静态发布器发布）

**如果缺少 TF**：
```bash
# 发布静态TF（示例：base_link到laser的变换）
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link laser_frame
```

---

### 步骤 3: 修改配置文件 ✓

#### 3.1 机器人参数
编辑 `src/nav_launcher/params/unitree_nav2_params.yaml`

```yaml
# 机器人尺寸（关键！）
robot_radius: 0.22  # 修改为你的机器人半径

# 速度限制（关键！）
max_vel_x: 0.5      # 最大线速度
max_vel_theta: 1.0  # 最大角速度
acc_lim_x: 2.5      # 线加速度
acc_lim_theta: 3.2  # 角加速度
```

- [ ] 已根据机器人实际尺寸修改 `robot_radius`
- [ ] 已设置合适的速度限制

#### 3.2 坐标系名称
如果你的坐标系名称与默认不同，需要修改：

```yaml
# AMCL
base_frame_id: "base_link"     # 你的机器人基座坐标系
global_frame_id: "map"
odom_frame_id: "odom"

# BT Navigator  
robot_base_frame: base_link

# Controller Server（多处）
# Planner Server（多处）
# 搜索并替换所有相关的坐标系名称
```

- [ ] 已检查并修改所有坐标系名称

#### 3.3 传感器话题
修改激光雷达话题名称：

```yaml
# AMCL
scan_topic: scan  # 你的激光话题名称（不带/）

# Local Costmap
scan:
  topic: /scan  # 你的激光话题（带/）
  
# Global Costmap  
scan:
  topic: /scan
```

- [ ] 已修改激光雷达话题名称
- [ ] 已修改里程计话题名称（如果不是 /odom）

---

### 步骤 4: 准备地图 ✓

#### 选项 A: 使用已有地图
```bash
# 将地图文件复制到 maps 目录
cp /path/to/your/map.pgm src/nav_launcher/maps/
cp /path/to/your/map.yaml src/nav_launcher/maps/
```

#### 选项 B: 使用 SLAM 建图
```bash
# 启动SLAM建图
ros2 launch nav_launcher unitree_navigation.launch.py slam:=True

# 遥控机器人移动
# （使用键盘遥控或手柄）

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/my_unitree_map

# 复制到项目
cp ~/my_unitree_map.* src/nav_launcher/maps/
```

- [ ] 地图文件已准备好
- [ ] 地图 YAML 文件配置正确

---

### 步骤 5: 构建和启动 ✓

```bash
# 1. 构建包
cd /home/ymc/ros2_nav_unitree_ws
colcon build --packages-select nav_launcher --symlink-install

# 2. Source 环境
source install/setup.zsh

# 3. 启动导航
ros2 launch nav_launcher unitree_navigation.launch.py \
    map:=$(ros2 pkg prefix nav_launcher)/share/nav_launcher/maps/my_map.yaml
```

- [ ] 包构建成功
- [ ] 启动文件运行无错误
- [ ] RVIZ 正常打开

---

### 步骤 6: 在 RVIZ 中测试 ✓

#### 6.1 设置初始位姿
1. 在 RVIZ 中点击 "2D Pose Estimate" 按钮
2. 在地图上机器人实际位置点击并拖动设置方向
3. 观察粒子云是否收敛

- [ ] 粒子云已收敛到机器人周围
- [ ] 激光扫描与地图匹配

#### 6.2 发送导航目标
1. 点击 "Nav2 Goal" 按钮
2. 在地图上点击目标位置
3. 观察机器人是否开始移动

- [ ] 路径规划成功（绿色路径显示）
- [ ] 机器人开始移动
- [ ] 机器人避障正常

---

## 🔍 故障排查

### 问题 1: 机器人不移动

**检查清单**：
```bash
# 1. 检查 cmd_vel 是否发布
ros2 topic echo /cmd_vel

# 2. 检查控制器状态
ros2 lifecycle get /controller_server

# 3. 检查速度是否为零
ros2 param get /controller_server FollowPath.max_vel_x
```

**可能原因**：
- [ ] 速度限制设置太小
- [ ] 机器人控制器未订阅 /cmd_vel
- [ ] 控制器节点未激活

### 问题 2: 定位不准确

**检查清单**：
```bash
# 1. 检查激光数据
ros2 topic hz /scan
ros2 topic echo /scan --once

# 2. 检查粒子数
ros2 param get /amcl max_particles

# 3. 查看定位可视化
# 在 RVIZ 中添加 PoseArray 显示：/particle_cloud
```

**可能原因**：
- [ ] 激光数据质量差
- [ ] 粒子数太少
- [ ] 运动模型参数不匹配
- [ ] 地图与实际环境不符

### 问题 3: 路径规划失败

**检查清单**：
```bash
# 1. 查看代价地图
# 在 RVIZ 中添加 Map 显示：
#   - /global_costmap/costmap
#   - /local_costmap/costmap

# 2. 检查规划器状态
ros2 lifecycle get /planner_server

# 3. 查看规划器日志
ros2 topic echo /planner_server/transition_event
```

**可能原因**：
- [ ] 机器人半径设置过大
- [ ] 目标点在障碍物内
- [ ] 代价地图膨胀过大
- [ ] 没有可行路径

### 问题 4: 碰撞风险

**调整参数**：
```bash
# 减小膨胀半径
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.3

# 增加机器人半径
ros2 param set /global_costmap/global_costmap robot_radius 0.25
```

---

## 📊 性能调优

### 调优清单

- [ ] **控制频率**：controller_frequency (建议 20Hz)
- [ ] **代价地图更新频率**：根据CPU能力调整
- [ ] **局部地图大小**：width/height (建议 3x3 米)
- [ ] **全局规划器**：选择合适算法
- [ ] **局部控制器**：选择合适算法
- [ ] **速度平滑**：启用 velocity_smoother

### 推荐配置（低配置机器人）
```yaml
controller_frequency: 10.0  # 降低频率
local_costmap:
  update_frequency: 3.0     # 降低更新频率
  width: 2                  # 减小地图尺寸
  height: 2
```

### 推荐配置（高配置机器人）
```yaml
controller_frequency: 20.0
local_costmap:
  update_frequency: 5.0
  width: 5
  height: 5
use_composition: True       # 使用组合节点
```

---

## 🎯 下一步优化

完成基础配置后，可以考虑：

1. **多传感器融合**
   - [ ] 添加深度相机
   - [ ] 添加IMU数据融合
   - [ ] 添加GPS（室外）

2. **高级功能**
   - [ ] 动态避障
   - [ ] 路径点导航
   - [ ] 自动充电
   - [ ] 多机器人协同

3. **参数优化**
   - [ ] DWB 控制器细调
   - [ ] AMCL 参数优化
   - [ ] 代价地图层权重调整

---

## 📞 获取帮助

如果遇到问题：

1. 查看 Nav2 官方文档：https://docs.nav2.org/
2. 搜索 Nav2 GitHub Issues
3. 查看日志：`ros2 topic echo /diagnostics`
4. 使用详细日志启动：`--log-level debug`

---

**祝你配置顺利！** 🎉

记住：导航调试需要耐心，一步一步来，先确保基础功能正常，再进行优化。
