# Triple Map Manager - KFS系统

一个基于ROS2的三重地图管理系统，支持Web界面交互式KFS标记放置和可视化。

## 🚀 快速开始

### 1️⃣ 克隆仓库

首先确保已经安装 ROS2 并创建了工作空间，然后将本仓库克隆到工作空间：

```bash
cd ~/ros2_ws/src
git clone https://github.com/fynngwu/triple_map_manager.git
```

### 2️⃣ 安装依赖

安装所需的依赖：

```bash
sudo apt install ros-humble-rclpy ros-humble-nav-msgs ros-humble-std-msgs \
  ros-humble-visualization-msgs ros-humble-geometry-msgs \
  ros-humble-rosbridge-suite \
  python3-numpy python3-opencv python3-yaml
```

**依赖库说明**：

- **ros-humble-rclpy**：ROS2 Python 客户端库，用于编写 ROS2 节点和服务
- **ros-humble-nav-msgs**：导航相关消息类型（如 OccupancyGrid），用于地图数据
- **ros-humble-std-msgs**：标准消息类型（String, Int32等）
- **ros-humble-visualization-msgs**：可视化消息（Marker, MarkerArray），用于在 RViz 中显示标记
- **ros-humble-geometry-msgs**：几何消息类型（Point, Pose等）
- **ros-humble-rosbridge-suite**：WebSocket 桥接套件，让 Web 界面与 ROS2 通信
- **python3-numpy**：Python 数值计算库，用于数组操作
- **python3-opencv**：OpenCV 计算机视觉库，用于图像处理
- **python3-yaml**：YAML 解析库，用于读取配置文件

### 3️⃣ 构建包

```bash
colcon build --packages-select triple_map_manager --symlink-install
```

### 4️⃣ 启动系统

```bash
source install/setup.bash
ros2 launch triple_map_manager kfs_console.launch.py
```

系统会自动完成以下操作：
1. **随机放置 KFS 标记**：3个 KFS1（蓝色）、4个 KFS2（红色）、1个 KFS Fake（灰色）
2. **发布到 ROS2**：自动发布 `/kfs_grid_data` 话题
3. **显示网格状态**：在终端中显示 ASCII 艺术网格布局
4. **保持运行**：节点保持活跃状态，持续监听 ROS2 系统

**优势**：
- ✅ 零配置，自动随机放置
- ✅ 适合 WSL2 环境
- ✅ 无需用户交互
- ✅ 自动遵循放置规则

**放置规则**：
- KFS1: 只能放在左/右列（第0列或第2列），最多3个
- KFS2: 不能放在第0行，最多4个
- KFS Fake: 不能放在第0行，最多1个

**查看结果**：在 RViz 中查看可视化标记

---

### 5️⃣ 配置说明

地图参数和障碍物配置在 `config/` 目录中，详见 [配置教程](doc/CONFIG_TUTORIAL.md)。

**发布的话题**：`/map1`, `/map2`, `/map3`（occupancy grid 地图），`/kfs_grid_data`（KFS网格数据）
## 📡 发布的话题

### 消息类型

本包发布的主要话题：

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/map1` | `nav_msgs/OccupancyGrid` | 第一个 occupancy grid 地图 |
| `/map2` | `nav_msgs/OccupancyGrid` | 第二个 occupancy grid 地图（用于 KFS 网格标记） |
| `/map3` | `nav_msgs/OccupancyGrid` | 第三个 occupancy grid 地图 |
| `/kfs_grid_data` | `std_msgs/String` | KFS 网格数据话题（由 Web 界面发布） |
| `/map2_kfs_markers` | `visualization_msgs/MarkerArray` | KFS 标记可视化 |

### OccupancyGrid 使用方式

**nav_msgs/OccupancyGrid** 消息类型用于表示占用栅格地图，常用于导航、SLAM等场景。

#### 常用操作：

```bash
# 1. 查看地图话题信息
ros2 topic info /map1

# 2. 查看地图消息内容
ros2 topic echo /map1 --once

# 3. 实时监听地图更新
ros2 topic echo /map1

# 4. 查看地图类型和数据类型
ros2 interface show nav_msgs/msg/OccupancyGrid
```

#### 在代码中使用：

```python
from nav_msgs.msg import OccupancyGrid

def map_callback(msg: OccupancyGrid):
    # 地图分辨率（米/像素）
    resolution = msg.info.resolution
    
    # 地图原点
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    
    # 地图尺寸
    width = msg.info.width    # 像素宽度
    height = msg.info.height  # 像素高度
    
    # 占用数据：-1=未知, 0-100=占用概率
    data = msg.data  # 一维数组
```

### KFS Grid Data 消息格式

**`/kfs_grid_data`** 话题发布 `std_msgs/String` 类型消息，内容为 JSON 格式：

```json
{
  "grid": [
    [0, 0, 0],
    [1, 0, 2],
    [0, 3, 0],
    [0, 1, 0]
  ],
  "timestamp": 1698234567890
}
```

#### 消息含义：

- **`grid`**：4×3 二维数组，表示 KFS 网格的标记布局
  - `0` = 空单元格
  - `1` = KFS1（蓝色标记）
  - `2` = KFS2（红色标记）
  - `3` = KFS Fake（灰色标记）
- **`timestamp`**：时间戳（毫秒）

#### 网格布局：

```
        Col 0    Col 1    Col 2
Row 0   [0,0]    [0,1]    [0,2]
Row 1   [1,0]    [1,1]    [1,2]
Row 2   [2,0]    [2,1]    [2,2]
Row 3   [3,0]    [3,1]    [3,2]
```

#### 使用示例：

```bash
# 监听 KFS 网格数据
ros2 topic echo /kfs_grid_data

# 查看消息类型
ros2 topic info /kfs_grid_data
ros2 interface show std_msgs/msg/String
```

### 配置文件

所有地图配置和障碍物定义在 `config/` 目录中，详见 [配置教程](doc/CONFIG_TUTORIAL.md)。

---

## 📋 项目概述

本项目实现了一个完整的三重地图管理系统，支持Web界面交互式KFS标记放置和可视化。  
详细的项目信息（系统架构、文件结构、技术特性等）请参阅 [项目详情](doc/PROJECT_DETAILS.md)。
