#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockMotionNode(Node):
    """模拟运动控制节点"""
    
    def __init__(self):
        super().__init__('mock_motion_node')
        self.logger = self.get_logger()
        
        # 订阅运动指令
        self.subscription = self.create_subscription(
            String,
            'motion_command',
            self.motion_callback,
            10)
            
        self.logger.info("Mock Motion Node Initialized. Listening on /motion_command...")

    def motion_callback(self, msg):
        command = msg.data
        self.logger.info(f"Received motion command: {command}")
        
        # 解析指令
        if command == "SAFETY_STOP":
            self.execute_stop()
        elif command == "MOVE_CHARGE":
            self.execute_charge()
        elif command == "MOVE_COME":
            self.execute_come()
        elif command == "MOVE_BACK":
            self.execute_back()
        else:
            self.logger.warning(f"Unknown motion command: {command}")

    def execute_stop(self):
        self.logger.info("ACTION: [EMERGENCY STOP] Stopping chassis, locking motors.")

    def execute_charge(self):
        self.logger.info("ACTION: [NAVIGATE] Navigating to charging station.")

    def execute_come(self):
        self.logger.info("ACTION: [FOLLOW] Locating user sound/position and approaching.")

    def execute_back(self):
        self.logger.info("ACTION: [MOVE] Moving backward 0.5 meters.")

def main(args=None):
    rclpy.init(args=args)
    node = MockMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
