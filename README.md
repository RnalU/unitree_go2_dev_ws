# Unitree GO2 ROS2 Navigation Workspace

## 简介 (Introduction)

本工作空间专为 **Unitree GO2** 机器狗开发，基于 **Ubuntu 20.04** 和 **ROS2 Foxy**。
项目集成了 **Navigation2** 导航框架，并针对宇树 GO2 的开发环境限制，实现了自定义的 **UDP 通讯桥接 (Bridge)** 方案，支持 C++ 和 Python 双版本。

## 环境要求 (Requirements)

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS2 Foxy Fitzroy
- **硬件平台**: Unitree GO2 (配备拓展坞)

## 主要功能 (Features)

1.  **Navigation2 导航集成**:
    - 包含完整的 Nav2 导航栈配置与启动文件。
    - 适配 GO2 的底盘控制与里程计。

2.  **SLAM 建图**:
    - 目前已集成 **Gmapping** 建图包。
    - 后续计划引入更多建图算法 (如 Cartographer,Point_Lio, SLAM Toolbox 等)。

3.  **自定义 UDP/Wi-Fi 桥接 (Custom Bridge)**:
    - **背景**: 宇树 GO2 不开放内部主板连接，仅提供拓展坞用于开发。在有线连接模式下，ROS2 DDS 的默认传输机制可能受限于单网卡或网络配置，导致通讯不畅。
    - **解决方案**: 开发了基于 UDP 的桥接包，绕过 DDS 限制，实现机器狗与主机之间的高效数据透传。
    - **双版本实现**: 提供 **C++** 和 **Python** 两种实现，可根据需求选择。
    - **架构**:
        - **Lower (下位机)**: 运行在机器狗拓展坞内部，**需要插入无线网卡，桥接包通过无线网卡进行通讯**。
        - **Host (上位机)**: 运行在开发者的电脑上。

## 目录结构 (Directory Structure)

- `src/go2_UDP_WIFI_bridge_*`: 自定义 UDP 桥接包 (Host/Lower, C++/Python)。
- `src/navigation2`: Navigation2 导航框架源码。
- `src/slam_gmapping`: Gmapping 建图算法。
- `src/nav_launcher`: 导航启动管理包。
- `src/unitree_*`: 宇树官方 SDK 及相关依赖 (目前主要使用自定义 Bridge)。
- `start_navigation.sh`: 导航启动与环境配置脚本。

## 安装与编译 (Installation)

在工作空间根目录下运行：

```bash
# 安装依赖 (如果需要)
rosdep install --from-paths src --ignore-src -r -y

# 编译工作空间
colcon build --symlink-install

# 加载环境变量
source install/setup.bash
```

## 使用说明 (Usage)

### 1. 桥接程序启动 (Bridge Setup)

为了实现主机与机器狗的通讯，需要在两端分别启动桥接程序。

#### A. 机器狗端 (Lower Side)
将 `src` 下的 `Lower` 相关包 (如 `go2_UDP_WIFI_bridge_Lower` 或 `go2_udp_wifi_bridge_lower_cpp`) 拷贝到机器狗拓展坞的系统中，编译并运行。

* **Python 版本:**
  ```bash
  ros2 launch go2_UDP_WIFI_bridge_Lower lower_bridge.launch.py
  ```
* **C++ 版本:**
  
  ```bash
  ros2 launch go2_udp_wifi_bridge_lower_cpp lower_bridge_cpp.launch.py
  ```

#### B. 主机端 (Host Side)
在开发本机上运行对应的 Host 节点。

* **Python 版本:**
  ```bash
  ros2 launch go2_UDP_WIFI_bridge_Host host_bridge.launch.py
  ```
* **C++ 版本:**
  ```bash
  ros2 launch go2_udp_wifi_bridge_host_cpp host_bridge_cpp.launch.py
  ```

### 2. 启动导航与建图 (Navigation & Mapping)

#### 启动导航
可以使用提供的脚本一键配置网络并启动：

```bash
./start_navigation.sh
```

或者手动启动 Launch 文件：
```bash
ros2 launch nav_launcher unitree_navigation.launch.py
```

#### 启动 Gmapping 建图
```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

## 常用脚本 (Scripts)

- **`start_navigation.sh`**: 
  - 自动配置 `ROS_DOMAIN_ID` (默认 42)。
  - 设置 `CYCLONEDDS_URI` 配置文件。
  - 检查与机器狗 (172.16.6.156) 的网络连通性。
  - 启动导航程序。

- **`setup_multi_robot.sh`**: 
  - 用于快速设置多机通讯的环境变量。

## 注意事项 (Notes)

- 确保主机与机器狗在同一网段，且能够互相 Ping 通。
- 默认机器狗 IP 设定为 `172.16.6.156`，主机 IP 设定为 `172.16.6.53` (可在脚本中修改)，该IP为WIFI路由器下的网段。
- 桥接包的 C++ 版本性能通常优于 Python 版本，建议在对延迟要求较高的场景下使用 C++ 版本。
