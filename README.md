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

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3️⃣ 构建包

```bash
colcon build --packages-select triple_map_manager --symlink-install
```

### 4️⃣ 启动系统

```bash
source install/setup.bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

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