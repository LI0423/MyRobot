#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试语音设置闹钟功能

该脚本模拟语音指令，发送到 voice_command 话题，测试 schedule_reminder 节点是否正确解析并设置闹钟。
"""

import rclpy
from rclpy.node import Node
from voice_msgs.msg import VoiceCommand
from life_assistance.msg import Reminder
import time
from datetime import datetime

class AlarmTestNode(Node):
    """测试闹钟功能的节点"""
    
    def __init__(self):
        super().__init__('alarm_test_node')
        
        # 发布语音指令
        self.voice_command_publisher = self.create_publisher(
            VoiceCommand, 'voice_command', 10)
        
        # 订阅提醒消息
        self.reminder_subscriber = self.create_subscription(
            Reminder, 'schedule_reminder', self.reminder_callback, 10)
        
        # 测试结果
        self.test_results = []
        
        # 测试用例
        self.test_cases = [
            "设置闹钟，明天8点30分",
            "设置闹钟，14:25",
            "设置闹钟，后天10点",
            "设置闹钟，9点"
        ]
        
        self.get_logger().info("闹钟测试节点初始化完成")
    
    def reminder_callback(self, msg):
        """处理提醒消息回调
        
        Args:
            msg (String): 提醒消息
        """
        message = msg.data
        self.get_logger().info(f"收到提醒消息: {message}")
        self.test_results.append(message)
    
    def run_tests(self):
        """运行测试用例"""
        self.get_logger().info("开始测试语音设置闹钟功能...")
        
        for i, command in enumerate(self.test_cases):
            self.get_logger().info(f"测试用例 {i+1}: {command}")
            
            # 创建语音指令消息
            voice_command_msg = VoiceCommand()
            voice_command_msg.command = command
            voice_command_msg.confidence = 0.95
            voice_command_msg.language = "zh-CN"
            
            # 发布消息
            self.voice_command_publisher.publish(voice_command_msg)
            self.get_logger().info(f"发布语音指令: {command}")
            
            # 等待3秒，让节点处理消息
            time.sleep(3)
        
        # 等待5秒，让节点完成处理
        time.sleep(5)
        
        # 打印测试结果
        self.get_logger().info("测试完成，结果如下:")
        for result in self.test_results:
            self.get_logger().info(f"- {result}")
        
        # 检查测试结果
        if len(self.test_results) == len(self.test_cases):
            self.get_logger().info("✅ 所有测试用例都成功设置了闹钟！")
        else:
            self.get_logger().info(f"❌ 只有 {len(self.test_results)} 个测试用例成功设置了闹钟，预期 {len(self.test_cases)} 个")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建测试节点
        test_node = AlarmTestNode()
        
        # 运行测试
        test_node.run_tests()
        
        # 销毁节点
        test_node.destroy_node()
    except Exception as e:
        print(f"测试过程中发生错误: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
