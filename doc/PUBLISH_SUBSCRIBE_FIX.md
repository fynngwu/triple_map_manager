# KFS Marker 发布订阅问题解决方案

## 问题描述

在使用 `kfs_console_node` 时，节点在启动时立即发布数据，但订阅者（`kfs_visualizer_node`）可能还没准备好连接，导致 marker 无法在 RViz2 中显示。

## 原因分析

这是典型的 ROS2 发布/订阅启动顺序问题：

1. **发布者过早发布**：`kfs_console_node` 在 `__init__` 中立即调用 `publish_grid()`
2. **订阅者延迟连接**：`kfs_visualizer_node` 需要时间启动并建立连接
3. **消息丢失**：默认 QoS 设置下，订阅者连接前发布的消息会丢失
4. **结果**：RViz2 中的 marker 无法显示

## 解决方案

我们使用 **QoS Transient Local** 机制来确保订阅者能够接收到数据。

### QoS Transient Local

**原理**：使用 `TRANSIENT_LOCAL` durability policy，让发布者"记住"最后一次消息。

**实现**：
- 发布者和订阅者都使用匹配的 QoS 配置
- 发布者保持最后一条消息
- 订阅者连接时自动接收最后一条消息
- 发布者只需发布一次即可

**代码示例**：
```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# 定义 QoS 配置（发布者和订阅者都要用相同的配置）
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # 关键！
)

# 发布者配置
self.grid_publisher = self.create_publisher(
    String,
    '/kfs_grid_data',
    qos_profile
)

# 订阅者配置（必须使用相同的 QoS）
self.grid_subscriber = self.create_subscription(
    String,
    '/kfs_grid_data',
    callback_function,
    qos_profile
)

# 重要：如果这个节点也发布消息给 RViz，也必须使用相同的 QoS
self.marker_publisher = self.create_publisher(
    MarkerArray,
    '/map2_kfs_markers',
    qos_profile
)
```

**优势**：
- ✅ ROS2 标准解决方案
- ✅ 适合"最后状态"类型的消息
- ✅ 即使启动顺序不同也能工作
- ✅ 代码简洁，只需发布一次
- ✅ 不需要额外的定时器或延迟

**限制**：
- ⚠️ 发布者和订阅者必须使用匹配的 QoS
- ⚠️ 消息必须是"最新状态"而不是历史记录

## 修改的文件

1. **`triple_map_manager/kfs_console_node.py`**
   - 添加 QoS TRANSIENT_LOCAL 配置到 publisher
   - 只发布一次数据

2. **`triple_map_manager/kfs_visualizer_node.py`**
   - 添加匹配的 QoS TRANSIENT_LOCAL 配置到 subscriber
   - **关键**：同时添加 TRANSIENT_LOCAL 配置到 marker publisher
   - 这确保 RViz2 即使在 visualizer 启动后也能收到 markers

## 其他可选方案

### 方案 A：延迟启动发布者

```python
# 在 launch 文件中延迟启动
kfs_console_node = Node(
    package='triple_map_manager',
    executable='kfs_console',
    name='kfs_console_main'
)

delayed_kfs = TimerAction(
    period=3.0,
    actions=[kfs_console_node]
)
```

**缺点**：需要硬编码等待时间，不够灵活。

### 方案 B：等待订阅者连接

```python
# 等待订阅者连接
while self.grid_publisher.get_subscription_count() == 0:
    time.sleep(0.1)

self.publish_grid()
```

**缺点**：可能导致节点无限等待，不推荐。

### 方案 C：使用服务调用（完全不同的架构）

将发布订阅改为服务调用：
- 订阅者准备好后调用服务
- 发布者返回数据

**缺点**：架构改动太大，不合适。

## 推荐方案

**最佳实践**：使用 **QoS Transient Local**

理由：
1. ✅ 符合 ROS2 设计理念
2. ✅ 代码简洁，易于维护
3. ✅ 不需要额外的定时器或延迟
4. ✅ 适合生产环境

## 测试验证

测试启动顺序是否正确：

```bash
# 1. 清理旧节点
./kill_kfs_nodes.sh

# 2. 启动系统
ros2 launch triple_map_manager kfs_console.launch.py

# 3. 检查日志
# 应该看到 "Received grid data" 在 "Published grid data" 之后

# 4. 检查 RViz2
# marker 应该能够正常显示
```

## 技术细节

### QoS Durability 类型

1. **VOLATILE（默认）**
   - 不保留历史消息
   - 订阅者只能接收连接后发布的消息

2. **TRANSIENT_LOCAL**
   - 发布者保留最后一条消息
   - 订阅者连接时自动接收最后一条消息
   - 适合"状态"类型的消息

### QoS 匹配规则

发布者和订阅者的 QoS 配置必须兼容：

| 发布者 | 订阅者 | 兼容性 |
|--------|--------|--------|
| TRANSIENT_LOCAL | TRANSIENT_LOCAL | ✅ 兼容 |
| TRANSIENT_LOCAL | VOLATILE | ❌ 不兼容 |
| VOLATILE | TRANSIENT_LOCAL | ✅ 兼容（订阅者要求更高） |
| VOLATILE | VOLATILE | ✅ 兼容 |

## 参考文献

- [ROS2 QoS Overview](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [DDS QoS Policies](https://www.omg.org/spec/DDS/1.4/PDF)

