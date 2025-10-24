# KFS Grid Interactive System - 测试指南

## 🎯 测试概述

KFS Grid Interactive System 是一个完整的基于Web的KFS标记交互系统，包含：
- Web交互界面（生产版本和测试版本）
- ROS2可视化节点
- 实时数据发布和可视化

## 📋 测试前准备

### 1. 系统要求检查
```bash
# 检查ROS2环境
echo $ROS_DISTRO
ros2 --version

# 检查Python包
python3 -c "import rclpy, visualization_msgs, std_msgs"
```

### 2. 编译和安装
```bash
# 进入工作空间
cd /home/wufy/ros2_ws

# 编译包
colcon build --packages-select triple_map_manager

# 安装包
source install/setup.bash
```

## 🚀 测试步骤

### 测试1: 独立Web界面测试（无需ROS2）

#### 1.1 打开测试版本
```bash
# 在浏览器中打开测试版本
firefox /home/wufy/ros2_ws/src/triple_map_manager/web/kfs_grid_test.html
# 或者
google-chrome /home/wufy/ros2_ws/src/triple_map_manager/web/kfs_grid_test.html
```

#### 1.2 测试功能
1. **选择标记类型**：
   - 点击 "KFS1 (Blue)" 按钮
   - 点击 "KFS2 (Red)" 按钮  
   - 点击 "KFS Fake (Gray)" 按钮

2. **放置标记**：
   - 在网格上点击单元格放置标记
   - 验证约束规则：
     - KFS1: 只能在最左列(0)和最右列(2)，最多3个
     - KFS2: 不能在第一行(0)，最多4个
     - KFS Fake: 不能在第一行(0)，最多1个

3. **测试随机放置**：
   - 点击 "Random Placement (Max)" 按钮
   - 验证所有标记都按规则放置

4. **测试清除功能**：
   - 点击 "Clear Grid" 按钮
   - 验证网格被清空

5. **测试发布功能**：
   - 点击 "Publish to ROS2 (Test)" 按钮
   - 查看控制台输出，验证数据格式

#### 1.3 预期结果
- 界面响应正常，无布局跳动
- 约束规则正确执行
- 控制台显示详细的操作日志
- 发布数据格式正确

### 测试2: 完整系统测试（包含ROS2）

#### 2.1 一键启动所有服务
```bash
# 启动所有服务（地图发布、KFS可视化、rosbridge、RViz）
ros2 launch triple_map_manager kfs_system.launch.py

# 或者选择性启动
ros2 launch triple_map_manager kfs_system.launch.py use_rviz:=false  # 不启动RViz
ros2 launch triple_map_manager kfs_system.launch.py use_rosbridge:=false  # 不启动rosbridge
ros2 launch triple_map_manager kfs_system.launch.py use_rviz:=false use_rosbridge:=false  # 只启动核心服务
```

在RViz中：
1. 添加 "Marker" 显示类型
2. 设置话题为 `/map2_kfs_markers`
3. 添加 "Map" 显示类型
4. 设置话题为 `/map2`

#### 2.2 打开生产版本Web界面
```bash
# 在浏览器中打开生产版本
firefox /home/wufy/ros2_ws/src/triple_map_manager/web/kfs_grid.html
```

#### 2.3 测试完整流程
1. **连接验证**：
   - 页面应显示 "Connected to ROS2"
   - 如果显示错误，检查rosbridge是否运行

2. **标记放置和可视化**：
   - 选择标记类型并放置
   - 点击 "Publish to ROS2"
   - 在RViz中应该看到：
     - 蓝色立方体 (KFS1)
     - 红色立方体 (KFS2)  
     - 灰色立方体 (KFS Fake)
   - 标记位置应该对应map2上的正确坐标

3. **坐标验证**：
   - Map2 origin: [3.2, 1.2, 0.0]
   - Grid resolution: 1.2米/单元格
   - 位置计算: x = 3.2 + col * 1.2, y = 1.2 + row * 1.2

#### 2.4 预期结果
- Web界面与ROS2成功连接
- 标记在RViz中正确显示
- 坐标映射准确
- 实时更新正常

### 测试3: 系统集成测试

#### 3.1 检查ROS2话题
```bash
# 检查话题列表
ros2 topic list | grep kfs

# 检查话题数据
ros2 topic echo /kfs_grid_data
ros2 topic echo /map2_kfs_markers
```

#### 3.2 检查节点状态
```bash
# 检查节点列表
ros2 node list

# 检查节点信息
ros2 node info /kfs_visualizer
ros2 node info /map_publisher
```

#### 3.3 性能测试
1. **快速放置测试**：
   - 快速连续放置多个标记
   - 验证系统响应性

2. **大量数据测试**：
   - 使用随机放置功能生成最大数量标记
   - 验证系统稳定性

3. **长时间运行测试**：
   - 让系统运行10-15分钟
   - 验证内存使用和稳定性

## 🔍 故障排除

### 常见问题

#### 1. 端口9090被占用
**症状**: `[Errno 98] Address already in use` 或 `Unable to start server`
**原因**: 之前的rosbridge进程没有完全关闭
**解决方案**:
```bash
# 使用清理脚本（推荐）
cd /home/wufy/ros2_ws/src/triple_map_manager/web
./cleanup_kfs.sh

# 或手动清理
pkill -f rosbridge_websocket
pkill -f map_publisher
pkill -f kfs_visualizer
pkill -f rviz2
```

#### 2. Web界面无法连接ROS2
**症状**: 页面显示 "Error connecting to ROS2"
**解决方案**:
```bash
# 检查rosbridge是否运行
ros2 node list | grep rosbridge

# 重启rosbridge
ros2 run rosbridge_server rosbridge_websocket
```

#### 3. 标记在RViz中不显示
**症状**: Web界面显示发布成功，但RViz中无标记
**解决方案**:
```bash
# 检查话题数据
ros2 topic echo /kfs_grid_data --once
ros2 topic echo /map2_kfs_markers --once

# 检查KFS可视化节点
ros2 node info /kfs_visualizer
```

#### 4. 坐标位置不正确
**症状**: 标记显示位置与预期不符
**解决方案**:
- 检查map2的origin设置
- 验证坐标映射计算
- 确认RViz中的坐标系设置

#### 5. 界面布局问题
**症状**: 按钮或网格显示异常
**解决方案**:
- 清除浏览器缓存
- 检查CSS样式加载
- 使用测试版本进行调试

#### 6. RViz OpenGL错误
**症状**: `GLSL link result` 或 `Vertex Program` 错误
**原因**: OpenGL着色器兼容性问题，通常不影响核心功能
**解决方案**:
```bash
# 方案1: 不启动RViz（推荐）
ros2 launch triple_map_manager kfs_system.launch.py use_rviz:=false

# 方案2: 使用核心功能测试脚本
cd /home/wufy/ros2_ws/src/triple_map_manager/web
./test_kfs_core.sh

# 方案3: 忽略错误（如果KFS标记能正常显示）
# 这个错误通常不影响KFS标记的可视化
```

## 📊 测试检查清单

### Web界面测试
- [ ] 页面正常加载
- [ ] 按钮响应正常
- [ ] 网格显示正确
- [ ] 标记放置功能正常
- [ ] 约束规则正确执行
- [ ] 随机放置功能正常
- [ ] 清除功能正常
- [ ] 计数显示正确
- [ ] 状态信息更新正常

### ROS2集成测试
- [ ] rosbridge连接成功
- [ ] 数据发布正常
- [ ] KFS可视化节点运行正常
- [ ] RViz显示正确
- [ ] 坐标映射准确
- [ ] 实时更新正常

### 系统稳定性测试
- [ ] 长时间运行稳定
- [ ] 内存使用正常
- [ ] 无崩溃或错误
- [ ] 性能表现良好

## 🎉 测试完成标准

当所有测试项目都通过时，系统被认为测试完成：

1. **功能完整性**: 所有核心功能正常工作
2. **集成稳定性**: ROS2集成稳定可靠
3. **用户体验**: 界面响应流畅，操作直观
4. **数据准确性**: 坐标映射和可视化准确
5. **系统稳定性**: 长时间运行无问题

## 📝 测试报告模板

```
测试日期: ___________
测试人员: ___________
测试环境: ___________

功能测试结果:
- Web界面: [ ] 通过 [ ] 失败
- ROS2集成: [ ] 通过 [ ] 失败  
- 可视化: [ ] 通过 [ ] 失败
- 系统稳定性: [ ] 通过 [ ] 失败

发现问题:
1. ________________
2. ________________
3. ________________

总体评价: [ ] 优秀 [ ] 良好 [ ] 需要改进

备注:
________________
```

---

**测试指南版本**: 1.0  
**创建日期**: 2025年10月23日  
**适用系统**: KFS Grid Interactive System v1.0
