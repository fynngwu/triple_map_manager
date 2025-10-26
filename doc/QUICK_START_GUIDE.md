# 🚀 KFS系统快速启动指南

## 📋 前置条件

确保您的系统已安装：
- ✅ ROS2 Humble
- ✅ Python 3.10+
- ✅ 现代Web浏览器
- ✅ RViz2

## 🎯 一键启动（推荐）

### 1. 进入项目目录
```bash
cd /home/wufy/ros2_ws/src/triple_map_manager
```

### 2. 启动完整系统
```bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

### 3. 等待系统启动
- 地图发布器启动
- KFS可视化器启动  
- ROSBridge服务器启动（端口9090）
- RViz2启动
- **2秒后**：Web界面自动打开

### 4. 开始使用
- 在Web界面的4×3网格中点击放置KFS标记
- 在RViz2中实时查看3D标记效果

## 🔧 故障排除

### 问题1：端口9090被占用
```bash
# 一键清理所有KFS节点
./kill_kfs_nodes.sh

# 重新启动
ros2 launch triple_map_manager kfs_direct.launch.py
```

### 问题2：Web界面无法连接
```bash
# 检查ROSBridge状态
ros2 topic echo /connected_clients

# 手动打开Web界面
xdg-open /home/wufy/ros2_ws/install/triple_map_manager/share/triple_map_manager/web/kfs_grid.html
```

### 问题3：重复节点警告
```bash
# 重启ROS2 daemon
ros2 daemon stop
ros2 daemon start

# 清理并重启
./kill_kfs_nodes.sh
ros2 launch triple_map_manager kfs_direct.launch.py
```

### 问题4：RViz2启动失败
```bash
# 检查环境变量
echo $DISPLAY

# 手动启动RViz2
rviz2 -d /home/wufy/ros2_ws/install/triple_map_manager/share/triple_map_manager/config/default.rviz
```

## 📊 系统状态检查

### 检查节点状态
```bash
ros2 node list
```
**期望输出**：
```
/kfs_visualizer_main
/map_publisher_main
/rosbridge_main
/rviz2
```

### 检查Topic状态
```bash
ros2 topic list | grep kfs
```
**期望输出**：
```
/kfs_grid_data
/map2_kfs_markers
```

### 检查Web连接
```bash
ros2 topic echo /connected_clients
```
**期望输出**：
```
clients:
- ip_address: ::1
  connection_time:
    sec: 1761283552
    nanosec: 919424795
---
```

## 🎮 使用流程

### 1. 启动系统
```bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

### 2. 等待完全加载
- 等待所有节点启动完成
- 等待Web界面自动打开（2秒延迟）
- 确认RViz2显示地图

### 3. 放置KFS标记
- 在Web界面选择标记类型：
  - **KFS1**：红色标记
  - **KFS2**：绿色标记  
  - **KFS Fake**：蓝色标记
- 点击网格位置放置标记
- 点击"发送"按钮

### 4. 查看效果
- 在RViz2中观察3D标记
- 标记应该出现在map2的对应位置
- 支持实时更新和删除

### 5. 清理系统
```bash
./kill_kfs_nodes.sh
```

## 🔍 调试模式

### 实时监控数据流
```bash
# 监控Web发送的数据
ros2 topic echo /kfs_grid_data

# 监控KFS可视化器发布的标记
ros2 topic echo /map2_kfs_markers
```

### 检查日志
```bash
# 查看KFS可视化器日志
ros2 node info /kfs_visualizer_main

# 查看地图发布器日志  
ros2 node info /map_publisher_main
```

## ⚡ 性能优化

### 1. 软件渲染（避免GPU问题）
RViz2已配置使用软件渲染，避免OpenGL兼容性问题。

### 2. 批量标记发布
使用MarkerArray实现高效批量发布，减少网络开销。

### 3. 延迟启动
Web界面延迟2秒启动，确保所有ROS2节点稳定运行。

## 🛠️ 高级用法

### 自定义地图
1. 替换`maps/`目录中的地图文件
2. 更新`config/maps_config.yaml`
3. 重新构建：`colcon build --packages-select triple_map_manager`

### 修改网格大小
1. 编辑`triple_map_manager/kfs_visualizer_node.py`
2. 修改`grid_rows`和`grid_cols`参数
3. 更新Web界面的JavaScript代码

### 添加新标记类型
1. 在`kfs_visualizer_node.py`中添加新类型
2. 在`kfs_grid.html`中添加对应按钮
3. 更新颜色和形状定义

## 📞 技术支持

### 常见错误码
- **Exit code 127**：找不到可执行文件
- **Exit code 134**：RViz2崩溃
- **Exit code -9**：进程被强制杀死

### 日志位置
```bash
# ROS2日志
ls ~/.ros/log/

# 系统日志
journalctl -u ros2
```

---

**快速启动完成！** 🎉  
如有问题，请参考故障排除部分或检查系统状态。
