#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SystemControlNode(Node):
    """系统与媒体控制节点"""
    
    def __init__(self):
        super().__init__('system_control_node')
        self.logger = self.get_logger()
        
        # 订阅系统指令
        self.sys_sub = self.create_subscription(
            String,
            'system_command',
            self.system_callback,
            10)
            
        # 订阅媒体指令
        self.media_sub = self.create_subscription(
            String,
            'media_command',
            self.media_callback,
            10)
            
        self.logger.info("System Control Node Initialized.")

    def system_callback(self, msg):
        command = msg.data
        self.logger.info(f"Received system command: {command}")
        
        if command == "SYS_PRIVACY_ON":
            self.logger.info("ACTION: [PRIVACY] Closing camera shutter, muting mic.")
        elif command == "SYS_PRIVACY_OFF":
            self.logger.info("ACTION: [PRIVACY] Opening camera shutter, unmuting mic.")
        elif command == "SYS_REBOOT":
            self.logger.info("ACTION: [SYSTEM] Initiating reboot sequence...")
        elif command == "NET_CONFIG":
            self.logger.info("ACTION: [NETWORK] Entering AP mode for configuration.")
        else:
            self.logger.warning(f"Unknown system command: {command}")

    def media_callback(self, msg):
        command_str = msg.data
        self.logger.info(f"Received media command: {command_str}")
        
        parts = command_str.split(':')
        intent = parts[0]
        slots = parts[1] if len(parts) > 1 else None
        
        if intent == "VOL_UP":
            self.logger.info(f"ACTION: [VOLUME] Increase volume. Level: {slots}")
        elif intent == "VOL_DOWN":
            self.logger.info(f"ACTION: [VOLUME] Decrease volume. Level: {slots}")
        elif intent == "VOL_MUTE":
            self.logger.info("ACTION: [VOLUME] Mute system.")
        elif intent == "MEDIA_CTRL":
            self.logger.info("ACTION: [MEDIA] Toggling play/pause or next track.")
        else:
            self.logger.warning(f"Unknown media command: {intent}")

def main(args=None):
    rclpy.init(args=args)
    node = SystemControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
