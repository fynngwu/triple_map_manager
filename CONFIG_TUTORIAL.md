# 配置教程

本教程将帮助您理解如何配置 `triple_map_manager` 的地图参数和障碍物设置。

## 📍 地图发布

本包发布三个 occupancy grid 地图话题：
- **`/map1`** - 第一个地图
- **`/map2`** - 第二个地图（用于KFS标记）
- **`/map3`** - 第三个地图

> **注意**：`map1` 的原点在 RViz2 中的右下角。

## 📝 配置文件位置

地图配置位于 `config/maps_config.yaml`  
障碍物配置位于 `config/obstacles_config.yaml`

## 🗺️ maps_config.yaml 配置说明

### 典型行作用解释

```yaml
map1:
  name: "map1"                    # 地图名称标识符
  resolution: 0.05                # 地图分辨率：0.05米/像素（5cm分辨率）
  width: 3.2                      # 地图宽度：3.2米（640个像素）
  height: 6.0                     # 地图高度：6.0米（1200个像素）
  origin: [0.0, 0.0, 0.0]        # 原点坐标：[x, y, theta]
```

#### 参数详解：

- **`name`**：地图的唯一标识名称
- **`resolution`**：每个像素代表的实际距离（单位：米）
  - 值越小，地图越精细
  - 推荐范围：0.01-0.1 米/像素
- **`width`**：地图宽度（单位：米）
  - 对应水平方向（x轴）
- **`height`**：地图高度（单位：米）
  - 对应垂直方向（y轴）
- **`origin`**：地图原点的三维坐标 `[x, y, theta]`
  - `x, y`：原点的位置偏移（单位：米）
  - `theta`：原点的旋转角度（单位：弧度）
  - 默认为 `[0.0, 0.0, 0.0]` 表示无偏移无旋转

### map1 配置示例

```yaml
map1:
  name: "map1"
  resolution: 0.05  # 高精度：5cm分辨率
  width: 3.2        # 3.2米宽 = 64像素
  height: 6.0       # 6.0米高 = 120像素
  origin: [0.0, 0.0, 0.0]
```

**用途**：适合精细操作和导航，原点在RViz2右下角。

### map2 配置示例（KFS网格）

```yaml
map2:
  name: "map2"
  resolution: 1.2   # 低精度：120cm分辨率（大格子）
  width: 4.8        # 4.8米宽 = 4个单元格
  height: 3.6       # 3.6米高 = 3个单元格
  origin: [3.2, 1.2, 0.0]  # 向右偏移3.2m，向上偏移1.2m
```

**用途**：KFS标记的4×3网格系统，每个格子1.2米×1.2米。

### map3 配置示例

```yaml
map3:
  name: "map3"
  resolution: 0.05  # 高精度：5cm分辨率
  width: 4.0        # 4.0米宽 = 80像素
  height: 6.0       # 6.0米高 = 120像素
  origin: [8.0, 0.0, 0.0]  # 向右偏移8米
```

**用途**：另一个精细地图区域。

## 🚧 obstacles_config.yaml 配置说明

### 典型行作用解释

```yaml
map1_obstacles:
  obstacles:
    - type: "filled"              # 障碍物类型：完全填充
      coordinates: [0.0, 0.0, 1.0, 1.0]  # [x1, y1, x2, y2]
```

#### 参数详解：

- **`type`**：障碍物类型
  - `"filled"`：完全填充的实体障碍物（黑色区域）
  - `"recover"`：可恢复区域（灰色区域）
- **`coordinates`**：矩形区域的坐标 `[x1, y1, x2, y2]`
  - `x1, y1`：矩形左下角坐标
  - `x2, y2`：矩形右上角坐标
  - 单位：米

### 障碍物类型对比

#### filled（填充障碍物）
```yaml
- type: "filled"
  coordinates: [2.0, 0.0, 2.5, 1.2]  # 占用的墙体
```
- **渲染**：黑色（占用）
- **用途**：实体墙体、机器、障碍物

#### recover（恢复区域）
```yaml
- type: "recover"
  coordinates: [0.0, 0.0, 1.0, 1.0]  # 可恢复区域
```
- **渲染**：灰色（未知/可恢复）
- **用途**：临时障碍、可清除区域

### 配置示例

**map1 障碍物**（走廊布局）：
```yaml
map1_obstacles:
  obstacles:
    # 恢复区域
    - type: "recover"
      coordinates: [0.0, 0.0, 1.0, 1.0]
    
    # 墙壁障碍
    - type: "filled"
      coordinates: [2.0, 0.0, 2.0, 1.2]  # 垂直墙体
    - type: "filled"
      coordinates: [2.0, 1.2, 3.2, 1.2]  # 水平墙体
```

**map2 障碍物**（KFS网格，通常为空）：
```yaml
map2_obstacles:
  obstacles: []  # KFS网格通常无障碍物
```

**map3 障碍物**：
```yaml
map3_obstacles:
  obstacles:
    - type: "filled"
      coordinates: [1.45, 1.5, 1.45, 6.0]  # 垂直障碍
    - type: "recover"
      coordinates: [3.0, 0.0, 4.0, 1.0]   # 恢复区域
```

## 🔧 修改配置步骤

1. **编辑地图配置**
   ```bash
   nano config/maps_config.yaml
   ```

2. **编辑障碍物配置**
   ```bash
   nano config/obstacles_config.yaml
   ```

3. **重新构建包**
   ```bash
   colcon build --packages-select triple_map_manager --symlink-install
   ```

4. **重新启动**
   ```bash
   ros2 launch triple_map_manager kfs_direct.launch.py
   ```

## 💡 最佳实践

1. **分辨率选择**
   - 精细操作：0.01-0.05 米/像素
   - 导航规划：0.05-0.1 米/像素
   - 大格子标记：1.0-2.0 米/像素

2. **原点和偏移**
   - 规划多个地图时，使用 `origin` 实现布局
   - map1 在原点 `[0, 0, 0]`
   - map2 向右偏移 `[3.2, 1.2, 0.0]`
   - map3 更右偏移 `[8.0, 0.0, 0.0]`

3. **障碍物坐标**
   - 所有坐标使用米为单位
   - 确保坐标在地图范围内
   - 避免重叠（除非故意）

## 📚 相关文档

- [ROS2 OccupancyGrid消息文档](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- [RViz2显示配置指南](../README.md#rviz2-集成)

