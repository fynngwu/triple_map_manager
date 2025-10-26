# Triple Map Manager - 项目详情

## 📋 项目概述

本项目实现了一个完整的三重地图管理系统，包含：
- **三重地图发布**：map1、map2、map3的独立管理
- **Web界面**：基于HTML/JavaScript的交互式KFS标记放置
- **实时可视化**：RViz2中的3D标记显示
- **ROS2桥接**：WebSocket连接实现Web-ROS2通信

## 🏗️ 系统架构

```
Web界面 (HTML/JS) 
    ↓ WebSocket
ROSBridge Server (port 9090)
    ↓ ROS2 Topics
KFS Visualizer Node
    ↓ MarkerArray
RViz2 (3D Visualization)
    ↑
Map Publisher Node (三重地图)
```

## 📁 文件结构

### 核心ROS2节点
- **`triple_map_manager/map_publisher_node.py`** - 三重地图发布器
  - 发布map1、map2、map3的occupancy grid
  - 加载YAML配置文件
  - 支持动态地图切换

- **`triple_map_manager/kfs_visualizer_node.py`** - KFS标记可视化器
  - 订阅`/kfs_grid_data` topic
  - 发布`/map2_kfs_markers` MarkerArray
  - 4×3网格坐标映射到map2坐标系

### Web界面
- **`web/kfs_grid.html`** - 主Web界面
  - 4×3交互式网格
  - KFS1、KFS2、KFS Fake标记类型
  - ROSLIB.js WebSocket连接
  - 实时数据发送到ROS2

### 配置文件
- **`config/maps_config.yaml`** - 地图配置
  - 地图文件路径
  - 坐标系设置
  - 分辨率参数
  - 详见 [配置教程](CONFIG_TUTORIAL.md)

- **`config/obstacles_config.yaml`** - 障碍物配置
  - 静态障碍物定义
  - 碰撞检测参数
  - 详见 [配置教程](CONFIG_TUTORIAL.md)

- **`config/default.rviz`** - RViz2配置
  - 3D显示设置
  - 地图和标记显示配置

### 地图文件
- **`maps/map1.pgm`** + **`maps/map1.yaml`** - 地图1
- **`maps/map2.pgm`** + **`maps/map2.yaml`** - 地图2（KFS标记目标）
- **`maps/map3.pgm`** + **`maps/map3.yaml`** - 地图3

### Launch文件
- **`launch/kfs_direct.launch.py`** - 主启动文件
  - 启动所有必要节点
  - 延迟2秒打开Web界面
  - 使用FindPackageShare动态路径

### 工具脚本
- **`kill_kfs_nodes.sh`** - 一键清理脚本
  - 精确杀死KFS相关节点
  - 清理端口9090
  - 状态验证

## 🔗 文件关联性

### 数据流
1. **Web → ROS2**：`kfs_grid.html` → ROSBridge → `/kfs_grid_data`
2. **ROS2 → 可视化**：`/kfs_grid_data` → `kfs_visualizer_node.py` → `/map2_kfs_markers` → RViz2
3. **地图发布**：`map_publisher_node.py` → `/map1`, `/map2`, `/map3` → RViz2

### 配置依赖
- `maps_config.yaml` → `map_publisher_node.py`
- `obstacles_config.yaml` → `map_publisher_node.py`
- `default.rviz` → RViz2启动
- `map*.pgm/yaml` → 地图发布

### 启动依赖
- `kfs_direct.launch.py` → 所有节点和Web界面
- `kill_kfs_nodes.sh` → 清理和重启

## 🎯 核心功能

### 1. 三重地图管理
- 独立的地图发布
- 动态地图切换
- 配置文件驱动

### 2. KFS标记系统
- 4×3网格布局
- 三种标记类型：KFS1、KFS2、KFS Fake
- 实时坐标映射

### 3. Web界面交互
- 点击式标记放置
- 实时数据同步
- 跨平台兼容

### 4. 3D可视化
- RViz2集成
- MarkerArray高效渲染
- 实时更新

## 🚀 启动方式

### 方法1：一键启动（推荐）
```bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

### 方法2：分步启动
```bash
# 1. 启动地图发布
ros2 run triple_map_manager map_publisher

# 2. 启动KFS可视化
ros2 run triple_map_manager kfs_visualizer

# 3. 启动ROSBridge
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090

# 4. 启动RViz2
rviz2 -d config/default.rviz

# 5. 打开Web界面
xdg-open web/kfs_grid.html
```

### 一键清理
```bash
./kill_kfs_nodes.sh
```

## 🔧 系统要求

- **ROS2 Humble**
- **Python 3.10+**
- **现代Web浏览器**
- **RViz2**
- **ROSBridge Server**

## 📊 技术特性

- **实时通信**：WebSocket + ROS2 Topics
- **高效渲染**：MarkerArray批量发布
- **动态配置**：YAML配置文件
- **跨平台**：Web界面兼容所有平台
- **模块化**：独立节点设计

## 🎮 使用说明

1. **启动系统**：运行launch文件
2. **等待加载**：系统自动打开Web界面
3. **放置标记**：在4×3网格中点击放置KFS标记
4. **实时查看**：在RViz2中观察3D标记
5. **清理重启**：使用kill脚本清理

## 🔍 故障排除

### 常见问题
- **端口9090被占用**：运行`./kill_kfs_nodes.sh`
- **Web界面无法连接**：检查ROSBridge状态
- **标记不显示**：检查RViz2配置
- **重复节点**：重启ROS2 daemon

### 调试命令
```bash
# 检查节点状态
ros2 node list

# 检查topic
ros2 topic list
ros2 topic echo /kfs_grid_data

# 检查连接
ros2 topic echo /connected_clients
```

## 📈 性能优化

- **MarkerArray**：批量发布减少网络开销
- **软件渲染**：避免GPU兼容性问题
- **延迟启动**：确保节点稳定启动
- **精确清理**：避免进程残留

## 🔮 未来扩展

- 支持更多地图类型
- 增加标记动画效果
- 实现标记历史记录
- 添加用户权限管理

---

**项目状态**：✅ 完成并可用  
**最后更新**：2025年10月24日  
**维护者**：wufy

