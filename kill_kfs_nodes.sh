#!/bin/bash

echo "=== 一键杀死KFS Direct Launch节点 ==="

# 定义要杀死的节点名称
NODES=("map_publisher_main" "kfs_visualizer_main" "rosbridge_main" "rviz2")

# 杀死函数
kill_node() {
    local node_name=$1
    echo "正在杀死节点: $node_name"
    
    # 查找进程ID - 使用更精确的匹配
    local pids=""
    case $node_name in
        "map_publisher_main")
            pids=$(pgrep -f "map_publisher.*map_publisher_main")
            ;;
        "kfs_visualizer_main")
            pids=$(pgrep -f "kfs_visualizer.*kfs_visualizer_main")
            ;;
        "rosbridge_main")
            pids=$(pgrep -f "rosbridge_websocket.*rosbridge_main")
            ;;
        "rviz2")
            pids=$(pgrep -f "rviz2")
            ;;
    esac
    
    if [ -z "$pids" ]; then
        echo "  ✅ 节点 $node_name 未运行"
        return 0
    fi
    
    # 杀死所有匹配的进程
    for pid in $pids; do
        echo "  杀死进程 $pid"
        kill -SIGINT $pid 2>/dev/null || true
    done
    
    # 等待进程终止
    sleep 2
    
    # 检查是否还有残留进程
    local remaining_pids=""
    case $node_name in
        "map_publisher_main")
            remaining_pids=$(pgrep -f "map_publisher.*map_publisher_main")
            ;;
        "kfs_visualizer_main")
            remaining_pids=$(pgrep -f "kfs_visualizer.*kfs_visualizer_main")
            ;;
        "rosbridge_main")
            remaining_pids=$(pgrep -f "rosbridge_websocket.*rosbridge_main")
            ;;
        "rviz2")
            remaining_pids=$(pgrep -f "rviz2")
            ;;
    esac
    
    if [ -n "$remaining_pids" ]; then
        echo "  ⚠️  节点 $node_name 仍有残留进程，强制杀死..."
        for pid in $remaining_pids; do
            echo "    强制杀死进程 $pid"
            kill -9 $pid 2>/dev/null || true
        done
        sleep 1
    fi
    
    # 最终检查
    local final_check=""
    case $node_name in
        "map_publisher_main")
            final_check=$(pgrep -f "map_publisher.*map_publisher_main")
            ;;
        "kfs_visualizer_main")
            final_check=$(pgrep -f "kfs_visualizer.*kfs_visualizer_main")
            ;;
        "rosbridge_main")
            final_check=$(pgrep -f "rosbridge_websocket.*rosbridge_main")
            ;;
        "rviz2")
            final_check=$(pgrep -f "rviz2")
            ;;
    esac
    
    if [ -z "$final_check" ]; then
        echo "  ✅ 节点 $node_name 已成功杀死"
        return 0
    else
        echo "  ❌ 节点 $node_name 杀死失败"
        return 1
    fi
}

# 杀死所有节点
success_count=0
total_count=${#NODES[@]}

for node in "${NODES[@]}"; do
    if kill_node "$node"; then
        ((success_count++))
    fi
done

echo ""
echo "=== 杀死结果 ==="
echo "成功杀死: $success_count/$total_count 个节点"

# 清理端口9090
echo ""
echo "=== 清理端口9090 ==="
PORT=9090
PORT_PID=$(netstat -tlnp 2>/dev/null | grep ":$PORT " | awk '{print $7}' | cut -d'/' -f1)

if [ -n "$PORT_PID" ]; then
    echo "端口$PORT被占用，正在清理..."
    kill -9 "$PORT_PID" 2>/dev/null || true
    sleep 1
    
    if netstat -tlnp 2>/dev/null | grep -q ":$PORT "; then
        echo "❌ 端口$PORT仍被占用"
    else
        echo "✅ 端口$PORT已释放"
    fi
else
    echo "✅ 端口$PORT未被占用"
fi

# 最终状态检查
echo ""
echo "=== 最终状态检查 ==="
cd /home/wufy/ros2_ws && source install/setup.bash 2>/dev/null
if ros2 node list 2>/dev/null | grep -E "(map_publisher_main|kfs_visualizer_main|rosbridge_main|rviz2)" >/dev/null; then
    echo "❌ 仍有KFS节点在运行:"
    ros2 node list 2>/dev/null | grep -E "(map_publisher_main|kfs_visualizer_main|rosbridge_main|rviz2)"
else
    echo "✅ 所有KFS节点已成功杀死"
fi

echo ""
echo "=== 脚本执行完成 ==="
