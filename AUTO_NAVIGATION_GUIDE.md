# ROS2导航自动化启动指南

## 核心改进

### 1. 自动设置初始位姿 ✅

**问题**: ROS2中AMCL不会自动发布`map->odom`变换,必须先手动设置初始位姿

**解决方案**: 新增`set_initial_pose`节点,在启动时自动发布初始位姿

### 2. Map Server QoS配置 ✅

**问题**: map_server使用Transient Local QoS,订阅者需要匹配的QoS才能接收地图

**状态**: Nav2默认配置已正确,global_costmap会自动订阅地图

---

## 使用方法

### 基本启动(自动设置初始位姿)

```bash
# 默认在地图原点(0,0,0)启动
ros2 launch nav_launcher unitree_navigation.launch.py
```

**这将自动**:
- 启动所有Nav2节点
- 等待3秒让AMCL初始化
- 自动发布初始位姿到`/initialpose`话题
- AMCL开始发布`map->odom`变换
- 机器人立即可以接收导航目标

### 指定初始位姿启动

```bash
# 在指定位置和朝向启动
ros2 launch nav_launcher unitree_navigation.launch.py \
    initial_pose_x:=2.0 \
    initial_pose_y:=1.5 \
    initial_pose_yaw:=1.57
```

**参数说明**:
- `initial_pose_x`: X坐标(米),默认0.0
- `initial_pose_y`: Y坐标(米),默认0.0  
- `initial_pose_yaw`: 朝向(弧度),默认0.0 (1.57≈90度, 3.14≈180度)

### 禁用自动初始位姿(手动设置)

```bash
# 如果你想在RVIZ中手动设置初始位姿
ros2 launch nav_launcher unitree_navigation.launch.py \
    auto_set_initial_pose:=false
```

然后在RVIZ中:
1. 点击工具栏的"2D Pose Estimate"按钮
2. 在地图上点击并拖动设置位置和朝向

---

## 工作流程

### 自动化流程
```
启动导航栈
    ↓
等待3秒(AMCL初始化)
    ↓
自动发布初始位姿到/initialpose
    ↓
AMCL接收初始位姿并开始定位
    ↓
AMCL发布map->odom变换
    ↓
✅ 机器人准备好接收导航目标
```

### TF树结构
```
map (全局地图坐标系)
 ↓ (AMCL发布,自动设置初始位姿后才发布)
odom (里程计坐标系)
 ↓ (机器人驱动发布)
base_footprint (机器人基座)
 ↓
base_link, sensors...
```

---

## 验证导航系统

### 1. 检查地图是否发布

```bash
# 检查/map话题
ros2 topic list | grep map

# 查看地图信息(应该看到发布者:map_server)
ros2 topic info /map

# 注意:在ROS2中,map使用Transient Local QoS
# 地图只在启动时发布一次,但订阅者可以获取历史数据
```

### 2. 检查TF树

```bash
# 安装tf2_tools(如果还没安装)
sudo apt install ros-foxy-tf2-tools

# 查看TF树
ros2 run tf2_tools view_frames

# 生成frames.pdf,检查map->odom->base_footprint链条是否完整
```

### 3. 检查AMCL状态

```bash
# 查看AMCL发布的位姿
ros2 topic echo /amcl_pose

# 查看粒子云
ros2 topic echo /particlecloud

# 如果能看到数据,说明AMCL正常工作
```

### 4. 检查map->odom变换

```bash
# 应该能看到实时变换数据
ros2 run tf2_ros tf2_echo map odom

# 如果报错"No transform",说明AMCL还没有初始位姿
# 使用自动初始位姿功能或手动设置
```

---

## 常见问题解决

### Q1: AMCL警告"Please set the initial pose"

**原因**: 没有设置初始位姿

**解决**:
```bash
# 方法1: 使用自动初始位姿(推荐)
ros2 launch nav_launcher unitree_navigation.launch.py

# 方法2: 手动发布初始位姿
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{
  header: {frame_id: "map"},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06854]
  }
}'
```

### Q2: RVIZ中看不到地图

**可能原因**:
1. RVIZ的Map显示插件使用了错误的话题
2. QoS设置不匹配

**解决**:
```bash
# 在RVIZ中
# 1. 添加Map显示插件
# 2. Topic设置为: /map
# 3. Durability Policy设置为: Transient Local
```

### Q3: 机器人实际位置与初始位姿不符

**原因**: 设置的初始位姿与机器人实际位置差异太大

**解决**:
```bash
# 1. 测量机器人在地图中的实际位置
# 2. 使用正确的初始位姿启动
ros2 launch nav_launcher unitree_navigation.launch.py \
    initial_pose_x:=<实际X> \
    initial_pose_y:=<实际Y> \
    initial_pose_yaw:=<实际朝向>

# 3. 或者禁用自动初始位姿,在RVIZ中手动设置
ros2 launch nav_launcher unitree_navigation.launch.py \
    auto_set_initial_pose:=false
```

### Q4: TF链断裂 - "frame 'base_link' not found"

**原因**: 机器人驱动没有发布`odom->base_footprint`变换

**检查**:
```bash
# 1. 确认机器人驱动正在运行
ros2 node list | grep -i unitree

# 2. 检查是否有odom话题
ros2 topic list | grep odom
ros2 topic echo /odom

# 3. 检查TF发布
ros2 topic echo /tf | grep base_footprint
```

**解决**: 确保Unitree机器人驱动正常运行并发布TF

---

## 完整启动示例

### 场景1: 室内办公室导航(已知起点)

```bash
# 机器人位于办公室入口(2.5, 1.0, 90度朝向)
ros2 launch nav_launcher unitree_navigation.launch.py \
    initial_pose_x:=2.5 \
    initial_pose_y:=1.0 \
    initial_pose_yaw:=1.57
```

### 场景2: 位置不确定(手动校准)

```bash
# 先禁用自动初始位姿
ros2 launch nav_launcher unitree_navigation.launch.py \
    auto_set_initial_pose:=false

# 在RVIZ中使用"2D Pose Estimate"工具手动设置
```

### 场景3: 使用不同的地图

```bash
ros2 launch nav_launcher unitree_navigation.launch.py \
    map:=/path/to/your/map.yaml \
    initial_pose_x:=0.0 \
    initial_pose_y:=0.0 \
    initial_pose_yaw:=0.0
```

---

## 高级配置

### 调整初始位姿延迟时间

编辑启动文件中的延迟参数:

```python
# unitree_navigation.launch.py
set_initial_pose_cmd = Node(
    ...
    parameters=[{
        ...
        'delay_sec': 5.0  # 从3.0改为5.0秒
    }]
)
```

### 调整初始位姿不确定性

编辑`set_initial_pose.py`中的协方差矩阵:

```python
# 增大协方差值 = 更高的不确定性 = AMCL会使用更多粒子搜索
msg.pose.covariance = [
    0.5, 0.0, ...,    # x方差:0.25 -> 0.5
    0.0, 0.5, ...,    # y方差:0.25 -> 0.5
    ...
    ..., 0.1          # yaw方差:0.06854 -> 0.1
]
```

---

## 导航测试步骤

### 1. 启动导航系统

```bash
ros2 launch nav_launcher unitree_navigation.launch.py \
    initial_pose_x:=0.0 \
    initial_pose_y:=0.0 \
    initial_pose_yaw:=0.0
```

### 2. 等待系统初始化

查看日志,确认:
- ✅ Map server加载地图
- ✅ AMCL接收地图
- ✅ 初始位姿已发布
- ✅ 无"Please set initial pose"警告

### 3. 在RVIZ中发送导航目标

1. 点击"Nav2 Goal"按钮
2. 在地图上点击目标位置
3. 观察机器人规划路径并移动

### 4. 监控导航状态

```bash
# 查看速度命令(应该重映射到/host/cmd_vel)
ros2 topic echo /host/cmd_vel

# 查看规划的路径
ros2 topic echo /plan

# 查看局部代价地图
ros2 topic echo /local_costmap/costmap
```

---

## 总结

通过添加**自动初始位姿设置**功能,现在你的导航系统可以:

✅ 一键启动,无需手动设置初始位姿  
✅ 支持指定初始位置和朝向  
✅ AMCL自动发布map->odom变换  
✅ 真正实现导航自动化  
✅ 兼容ROS2 Foxy的所有特性  

如果需要更精确的初始定位,可以:
- 禁用自动初始位姿
- 在RVIZ中手动设置
- 或使用更精确的初始位姿参数
