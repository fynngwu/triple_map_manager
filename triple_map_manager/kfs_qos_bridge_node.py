#!/usr/bin/env python3
"""
KFS QoS Bridge Node

功能：
- 订阅来自 rosbridge/Web 的 /kfs_grid_data 消息（使用默认 QoS，兼容任何 QoS）
- 以 TRANSIENT_LOCAL QoS 重新发布到 /kfs_grid_data
- 确保使用 TRANSIENT_LOCAL QoS 的订阅者能够接收到消息

这个节点解决了 rosbridge 发布的消息 QoS 不匹配的问题。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class KFSQoSBridgeNode(Node):
    """KFS QoS 桥接节点"""
    
    def __init__(self):
        super().__init__('kfs_qos_bridge')
        
        # 订阅者：使用默认 QoS（兼容来自 rosbridge 的任何 QoS）
        # 订阅来自 rosbridge/Web 的原始消息
        self.subscriber = self.create_subscription(
            String,
            '/kfs_grid_data_raw',  # 从 rosbridge 接收的原始话题
            self.callback,
            10  # 默认 QoS（VOLATILE），兼容任何发布者
        )
        
        # 发布者：使用 TRANSIENT_LOCAL QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.publisher = self.create_publisher(
            String,
            '/kfs_grid_data',  # 发布到标准话题（使用 TRANSIENT_LOCAL QoS）
            qos_profile
        )
        
        self.get_logger().info('KFS QoS Bridge Node started')
        self.get_logger().info('Subscribing to /kfs_grid_data_raw (from rosbridge, any QoS)')
        self.get_logger().info('Publishing to /kfs_grid_data (with TRANSIENT_LOCAL QoS)')
    
    def callback(self, msg):
        """接收消息并重新发布"""
        self.publisher.publish(msg)
        self.get_logger().info('Bridged kfs_grid_data message (QoS: any -> TRANSIENT_LOCAL)')


def main(args=None):
    rclpy.init(args=args)
    node = KFSQoSBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

