# Unitree Topic Relay Package

## 📋 概述

这个包用于在宇树 Go2 机器狗的扩展板上运行，将内部主板发布的话题转发到扩展板网络，使得通过WiFi连接的远程笔记本电脑能够接收这些话题。

### 为什么需要话题转发？

**网络拓扑：**
```
笔记本电脑 <--WiFi--> 扩展板 <--网线--> 内部主板
```

- 内部主板通过网线与扩展板通信
- 笔记本通过WiFi与扩展板通信
- 由于网络隔离，笔记本无法直接接收内部主板的话题
- 扩展板可以接收内部主板的话题，因此在扩展板上运行转发程序

---

## 🎯 ROS2关键概念解析

### 1. DDS通信机制

**ROS1 vs ROS2:**

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 通信机制 | Master-based | DDS (Data Distribution Service) |
| 网络发现 | 需要ROS_MASTER_URI | 自动发现（多播/单播） |
| 跨网络 | 困难 | 相对容易（配置DDS） |

**ROS2使用DDS的优势：**
- 无需中心Master节点
- 支持多种QoS配置
- 更好的实时性能
- 原生支持跨网络通信

### 2. QoS (Quality of Service)

ROS2引入了QoS配置，这是与ROS1最大的不同之一：

```python
# 传感器数据QoS（高频，允许丢包）
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力而为
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# 控制指令QoS（低频，必须可靠）
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**QoS策略说明：**
- **BEST_EFFORT**: 适合激光雷达、相机等高频传感器数据，偶尔丢包不影响
- **RELIABLE**: 适合控制指令、配置参数等关键数据，必须保证送达

### 3. 节点参数管理

**ROS1:**
```python
# 全局参数服务器
rospy.get_param('/my_param', default_value)
rospy.set_param('/my_param', value)
```

**ROS2:**
```python
# 节点本地参数
self.declare_parameter('my_param', default_value)
value = self.get_parameter('my_param').value
```

**关键区别：**
- ROS1有全局参数服务器，所有节点共享
- ROS2参数是节点本地的，更安全、更模块化

---

## 📦 话题转发映射

### Navigation所需话题

| 功能 | 宇树原始话题（示例） | 标准Nav2话题 | 消息类型 |
|------|---------------------|--------------|----------|
| 激光雷达 | `/utlidar/scan` | `/scan` | sensor_msgs/LaserScan |
| 里程计 | `/utlidar/robot_odom` | `/odom` | nav_msgs/Odometry |
| IMU | `/utlidar/imu` | `/imu` | sensor_msgs/Imu |
| TF变换 | `/tf` | `/tf_relay` | tf2_msgs/TFMessage |
| 静态TF | `/tf_static` | `/tf_static_relay` | tf2_msgs/TFMessage |
| 速度指令 | `/cmd_vel` | `/cmd_vel_unitree` | geometry_msgs/Twist |

**⚠️ 重要提示：** 
宇树的实际话题名称可能不同，需要在扩展板上使用 `ros2 topic list` 查看确切的话题名称！

---

## 🚀 使用步骤

### 步骤1: 查看宇树实际话题

在扩展板上执行：

```bash
# SSH连接到扩展板
ssh unitree@<扩展板IP>

# 查看所有话题
ros2 topic list

# 查看特定话题的类型
ros2 topic info /your_topic_name

# 查看话题内容（实时）
ros2 topic echo /your_topic_name
```

**需要确认的话题：**
- [ ] 激光雷达话题名称和类型
- [ ] 里程计话题名称和类型
- [ ] IMU话题名称和类型
- [ ] 速度控制话题名称和类型

### 步骤2: 修改配置文件

根据步骤1查到的实际话题名称，编辑配置文件：

```bash
# 在笔记本上编辑
cd ~/ros2_nav_unitree_ws/src/unitree_topic_relay/config
nano topic_relay_params.yaml
```

修改对应的 `*_input_topic` 参数为实际的宇树话题名称。

### 步骤3: 编译包

在笔记本上：

```bash
cd ~/ros2_nav_unitree_ws
colcon build --packages-select unitree_topic_relay
source install/setup.bash
```

### 步骤4: 部署到扩展板

将编译好的包复制到扩展板：

```bash
# 方法1: 使用scp
scp -r install/unitree_topic_relay unitree@<扩展板IP>:~/ros2_ws/install/

# 方法2: 在扩展板上直接编译
# SSH到扩展板后
cd ~/ros2_ws/src
# 将源码复制过来
colcon build --packages-select unitree_topic_relay
source install/setup.bash
```

### 步骤5: 在扩展板上运行

```bash
# SSH连接到扩展板
ssh unitree@<扩展板IP>

# 设置环境变量（如果需要）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds_config.xml

# 启动转发节点
ros2 launch unitree_topic_relay topic_relay.launch.py

# 或者使用自定义配置
ros2 launch unitree_topic_relay topic_relay.launch.py \
    params_file:=/path/to/custom_params.yaml
```

### 步骤6: 在笔记本上验证

在笔记本上检查是否能接收到转发的话题：

```bash
# 列出所有话题
ros2 topic list

# 检查转发的话题
ros2 topic echo /scan
ros2 topic echo /odom

# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /odom
```

---

## 🔧 故障排查

### 问题1: 笔记本收不到话题

**可能原因：**
1. DDS配置不正确
2. 网络防火墙阻止
3. 话题名称配置错误

**解决方案：**

```bash
# 1. 检查DDS配置
echo $RMW_IMPLEMENTATION  # 应该是 rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI

# 2. 在扩展板上查看转发节点是否正常
ros2 node list  # 应该看到 unitree_topic_relay_node
ros2 topic list  # 检查输出话题是否存在

# 3. 查看节点日志
ros2 run unitree_topic_relay topic_relay_node --ros-args --log-level debug

# 4. 检查网络连接
ping <扩展板IP>
```

### 问题2: 话题类型不匹配

**错误信息：**
```
[WARN] Topic type mismatch
```

**解决方案：**

```bash
# 在扩展板上查看宇树话题的确切类型
ros2 topic info /utlidar/scan
ros2 interface show sensor_msgs/msg/LaserScan

# 如果类型不同，需要修改节点代码中的消息类型
```

### 问题3: 转发延迟高

**可能原因：**
- QoS配置不当
- 网络带宽不足
- 消息频率过高

**解决方案：**

```bash
# 1. 降低传感器发布频率（在宇树驱动端）
# 2. 调整QoS配置（修改节点代码）
# 3. 使用有线连接代替WiFi（如果可能）
```

---

## 📚 进阶配置

### 添加自定义话题转发

如果需要转发其他话题，修改 `topic_relay_node.py`：

```python
# 1. 在 declare_parameters() 中添加参数
self.declare_parameter('custom_input_topic', '/custom/input')
self.declare_parameter('custom_output_topic', '/custom/output')
self.declare_parameter('enable_custom_relay', True)

# 2. 在 setup_relays() 中添加初始化
if self.get_parameter('enable_custom_relay').value:
    self.setup_custom_relay()

# 3. 实现转发函数
def setup_custom_relay(self):
    input_topic = self.get_parameter('custom_input_topic').value
    output_topic = self.get_parameter('custom_output_topic').value
    
    from your_msgs.msg import CustomMsg
    
    self.custom_pub = self.create_publisher(
        CustomMsg, output_topic, self.sensor_qos
    )
    
    self.custom_sub = self.create_subscription(
        CustomMsg, input_topic, self.custom_callback, self.sensor_qos
    )
    
    self.get_logger().info(f'✓ 自定义话题转发: {input_topic} -> {output_topic}')

def custom_callback(self, msg):
    self.custom_pub.publish(msg)
```

### 话题过滤和处理

如果需要在转发时修改消息内容：

```python
def lidar_callback(self, msg):
    # 修改消息内容
    modified_msg = msg
    modified_msg.range_max = 10.0  # 限制最大范围
    
    # 发布修改后的消息
    self.lidar_pub.publish(modified_msg)
```

---

## 📖 相关文档

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [Nav2文档](https://navigation.ros.org/)
- [CycloneDDS配置](https://github.com/eclipse-cyclonedds/cyclonedds)
- [宇树SDK文档](https://github.com/unitreerobotics)

---

## 🤝 贡献

如果发现bug或有改进建议，请提出issue或pull request。

---

## 📝 许可证

Apache-2.0

---

## ⚠️ 注意事项

1. **安全性**: 转发节点应该只在扩展板上运行，不要在笔记本上运行
2. **网络带宽**: 转发大量高频数据可能占用较多带宽
3. **延迟**: 转发会引入额外的网络延迟，通常在几毫秒到几十毫秒
4. **消息类型**: 确保源话题和目标话题使用相同的消息类型
5. **DDS配置**: 确保扩展板和笔记本使用相同的DDS配置
