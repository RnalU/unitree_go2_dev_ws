# 地图文件说明

## 如何使用地图

### 方式1：使用已有地图
如果你已经有地图文件（.pgm 和 .yaml），将它们复制到此目录。

### 方式2：使用SLAM建图
1. 启动SLAM建图模式：
```bash
ros2 launch nav_launcher unitree_navigation.launch.py slam:=True
```

2. 遥控机器人移动，构建地图

3. 保存地图：
```bash
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix nav_launcher)/share/nav_launcher/maps/my_new_map
```

### 方式3：从ROS1迁移地图
直接将ROS1的地图文件（.pgm和.yaml）复制到此目录即可使用。

## 地图格式

### PGM文件
- 灰度图像，表示占用栅格地图
- 黑色（0）= 障碍物
- 白色（255）= 可通行区域
- 灰色 = 未知区域

### YAML文件
包含地图元数据：
- `image`: 图像文件名
- `resolution`: 分辨率（米/像素）
- `origin`: 地图原点坐标
- `occupied_thresh`: 占用阈值
- `free_thresh`: 空闲阈值
- `negate`: 是否反转颜色

## 测试地图
在RVIZ中查看地图：
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
ros2 run rviz2 rviz2
```

然后在RVIZ中添加Map显示，话题选择 `/map`。
