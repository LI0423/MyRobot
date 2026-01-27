#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音交互测试脚本
用于测试小智助手与其他模块的交互
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from voice_msgs.msg import VoiceCommand
from voice_msgs.srv import TextToSpeech
import time

class VoiceInteractionTester(Node):
    """语音交互测试节点"""
    
    def __init__(self):
        super().__init__('voice_interaction_tester')
        
        # 发布语音结果
        self.voice_result_pub = self.create_publisher(String, 'voice_result', 10)
        # 订阅语音命令
        self.voice_command_sub = self.create_subscription(VoiceCommand, 'voice_command', self.voice_command_callback, 10)
        # 调用文本到语音服务
        self.tts_client = self.create_client(TextToSpeech, 'text_to_speech')
        
        self.logger.info("语音交互测试节点已启动")
    
    def voice_command_callback(self, msg):
        """处理语音命令"""
        self.logger.info(f"收到语音命令: {msg.command}, 置信度: {msg.confidence}")
    
    def test_wake_up(self):
        """测试唤醒功能"""
        self.logger.info("测试唤醒功能...")
        msg = String()
        msg.data = "小智"
        self.voice_result_pub.publish(msg)
        time.sleep(1)
    

    
    def test_vision_command(self):
        """测试视觉感知命令"""
        self.logger.info("测试视觉感知命令...")
        msg = String()
        msg.data = "你看到了什么"
        self.voice_result_pub.publish(msg)
        time.sleep(2)
    
    def test_life_command(self):
        """测试生活辅助命令"""
        self.logger.info("测试生活辅助命令...")
        msg = String()
        msg.data = "明天天气怎么样"
        self.voice_result_pub.publish(msg)
        time.sleep(2)
    
    def test_entertainment_command(self):
        """测试娱乐互动命令"""
        self.logger.info("测试娱乐互动命令...")
        msg = String()
        msg.data = "我们来玩个游戏吧"
        self.voice_result_pub.publish(msg)
        time.sleep(2)
    
    def test_home_command(self):
        """测试家居控制命令"""
        self.logger.info("测试家居控制命令...")
        msg = String()
        msg.data = "开灯"
        self.voice_result_pub.publish(msg)
        time.sleep(2)
    
    def test_sleep(self):
        """测试休眠功能"""
        self.logger.info("测试休眠功能...")
        msg = String()
        msg.data = "休眠"
        self.voice_result_pub.publish(msg)
        time.sleep(1)
    
    def test_all_commands(self):
        """测试所有命令"""
        self.test_wake_up()
        self.test_vision_command()
        self.test_life_command()
        self.test_entertainment_command()
        self.test_home_command()
        self.test_sleep()
    
    def speak(self, text):
        """测试语音合成"""
        if self.tts_client.wait_for_service(timeout_sec=1.0):
            request = TextToSpeech.Request()
            request.text = text
            request.voice_type = "female"
            future = self.tts_client.call_async(request)
            self.logger.info(f"请求语音合成: {text}")
        else:
            self.logger.error("文本到语音服务不可用")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    tester = VoiceInteractionTester()
    
    try:
        # 测试所有功能
        tester.test_all_commands()
        
        # 保持节点运行
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.logger.info("收到中断信号，停止测试")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()