#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from voice_msgs.msg import VoiceCommand

# 添加项目根目录到Python路径
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../../../..')

# 导入工具模块
from utils.logger import logger_manager

# 导入其他必要的模块
import re
from datetime import datetime, timedelta

class ScheduleReminderNode(Node):
    """日程提醒节点"""
    
    def __init__(self):
        super().__init__('schedule_reminder')
        
        # 初始化名称和状态
        self.name = "生活辅助模块"
        self.status = "未初始化"
        
        # 使用自定义日志管理器
        self.logger = logger_manager.get_ros2_logger(self.name)
        
        # 配置参数
        self.config = {
            "schedule_reminder_enabled": True,
            "health_monitoring_enabled": True,
            "medication_reminder_enabled": True,
            "emergency_contact": "13800000000" # 示例紧急联系人
        }
        
        # 发布提醒话题
        self.reminder_publisher = self.create_publisher(
            String, 'schedule_reminder', 10)
        
        # 发布紧急求助话题
        self.sos_publisher = self.create_publisher(
            String, 'emergency_sos', 10)
        
        # 订阅 clock 话题
        self.clock_subscriber = self.create_subscription(
            Clock, 'clock', self.clock_callback, 10)
        
        # 订阅 voice_command 话题
        self.voice_command_subscriber = self.create_subscription(
            VoiceCommand, 'voice_command', self.voice_command_callback, 10)
        
        # 定时器，每分钟检查一次日程
        self.timer = self.create_timer(60.0, self.check_schedule)
        
        # 初始化闹钟列表
        self.alarms = []
        
        # 初始化用药提醒列表
        self.medication_reminders = []
        
        # 初始化当前时间
        self.current_time = datetime.now()
        
        # 初始化模块
        self.initialize()
    
    def clock_callback(self, msg):
        """处理 clock 话题的回调
        
        Args:
            msg (Clock): 时钟消息
        """
        # 从消息中获取时间并更新当前时间
        time_msg = msg.clock
        self.current_time = datetime.fromtimestamp(time_msg.sec + time_msg.nanosec / 1e9)
        # 每秒检查一次闹钟
        self.check_alarms()
    
    def initialize(self):
        """初始化生活辅助模块"""
        self.logger.info(f"正在初始化{self.name}...")
        self.status = "初始化中"
        
        try:
            # 初始化日程管理
            self._init_schedule_manager()
            
            # 初始化健康监测
            self._init_health_monitoring()
            
            # 初始化用药提醒
            self._init_medication_reminder()
            
            # 初始化紧急求助
            self._init_emergency()
            
            # 初始化完成
            self.status = "运行中"
            self.logger.info(f"{self.name}初始化完成！")
            
        except Exception as e:
            self.status = "初始化失败"
            self.logger.error(f"{self.name}初始化失败: {str(e)}")
            raise
    
    def _init_schedule_manager(self):
        """初始化日程管理"""
        self.logger.info("初始化日程管理...")
        # TODO: 实现日程管理初始化逻辑
        
    def _init_health_monitoring(self):
        """初始化健康监测"""
        self.logger.info("初始化健康监测...")
        # TODO: 实现健康监测初始化逻辑
        
    def _init_medication_reminder(self):
        """初始化用药提醒"""
        self.logger.info("初始化用药提醒...")
        # 示例：添加一个默认的用药提醒（如果列表为空）
        if not self.medication_reminders:
            morning_time = datetime.now().replace(hour=8, minute=0, second=0, microsecond=0)
            if morning_time < datetime.now():
                morning_time += timedelta(days=1)
            
            self.add_medication_reminder(morning_time, "降压药")
            self.logger.info(f"已添加示例用药提醒: {morning_time.strftime('%Y-%m-%d %H:%M')} 吃降压药")

    def _init_emergency(self):
        """初始化紧急求助"""
        self.logger.info("初始化紧急求助...")
        # 可以在这里检查SOS硬件按钮状态等
    
    def voice_command_callback(self, msg):
        """处理 voice_command 话题的回调
        
        Args:
            msg (VoiceCommand): 语音指令消息
        """
        command = msg.command
        confidence = msg.confidence
        language = msg.language
        
        self.logger.info(f"收到语音指令: {command}, 置信度: {confidence}, 语言: {language}")
        
        # 检查置信度，只有置信度高于0.8的指令才处理
        if confidence < 0.8:
            self.logger.warning(f"语音指令置信度低 ({confidence})，忽略")
            return
        
        # 处理 SOS 指令
        if "SOS" in command or "救命" in command or "紧急" in command:
            self.trigger_sos(f"语音触发: {command}")
            return
            
        # 处理 ALARM_STOP 指令
        if command == "ALARM_STOP":
            self.stop_all_alerts()
            return

        # 处理结构化指令 (适配 XiaoZhi 节点发来的 params)
        if command.startswith("set_medication:"):
            # set_medication:time:medicine_name
            try:
                parts = command.split(":")
                time_str = parts[1] # HHMM
                medicine = parts[2] if len(parts) > 2 else "药物"
                
                now = datetime.now()
                hour = int(time_str[:2])
                minute = int(time_str[2:])
                target_time = now.replace(hour=hour, minute=minute, second=0, microsecond=0)
                if target_time < now:
                    target_time += timedelta(days=1)
                
                self.add_medication_reminder(target_time, medicine)
            except Exception as e:
                self.logger.error(f"解析用药指令失败: {e}")
            return

        # 解析语音指令，提取闹钟时间信息
        alarm_time = self.parse_alarm_time(command)
        if alarm_time:
            # 设置闹钟
            self.set_alarm(alarm_time, command)
        else:
            self.logger.info(f"未识别到闹钟时间信息: {command}")
    
    def parse_alarm_time(self, command):
        """解析语音指令，提取闹钟时间信息
        
        Args:
            command (str): 语音指令
        
        Returns:
            datetime or None: 解析出的闹钟时间，如果未识别到则返回None
        """
        # 转换为小写，方便匹配
        command_lower = command.lower()
        
        # 匹配 "设置闹钟" 关键词
        if "闹钟" not in command_lower:
            return None
        
        # 匹配时间格式，如 "8点30分"、"14:25"、"明天9点"、"后天10点30分"
        
        # 匹配 "X点Y分" 格式
        pattern1 = r'(\d+)点(\d+)?分?'
        match1 = re.search(pattern1, command_lower)
        if match1:
            hour = int(match1.group(1))
            minute = int(match1.group(2)) if match1.group(2) else 0
            
            # 检查是否有 "明天"、"后天" 关键词
            if "明天" in command_lower:
                # 明天的时间
                target_date = datetime.now() + timedelta(days=1)
            elif "后天" in command_lower:
                # 后天的时间
                target_date = datetime.now() + timedelta(days=2)
            else:
                # 今天的时间
                target_date = datetime.now()
            
            # 构建闹钟时间
            alarm_time = target_date.replace(hour=hour, minute=minute, second=0, microsecond=0)
            
            # 如果今天的时间已经过了，则设置为明天的时间
            if alarm_time < datetime.now() and "明天" not in command_lower and "后天" not in command_lower:
                alarm_time += timedelta(days=1)
            
            return alarm_time
        
        # 匹配 "X:Y" 格式
        pattern2 = r'(\d+):(\d+)'
        match2 = re.search(pattern2, command_lower)
        if match2:
            hour = int(match2.group(1))
            minute = int(match2.group(2))
            
            # 检查是否有 "明天"、"后天" 关键词
            if "明天" in command_lower:
                # 明天的时间
                target_date = datetime.now() + timedelta(days=1)
            elif "后天" in command_lower:
                # 后天的时间
                target_date = datetime.now() + timedelta(days=2)
            else:
                # 今天的时间
                target_date = datetime.now()
            
            # 构建闹钟时间
            alarm_time = target_date.replace(hour=hour, minute=minute, second=0, microsecond=0)
            
            # 如果今天的时间已经过了，则设置为明天的时间
            if alarm_time < datetime.now() and "明天" not in command_lower and "后天" not in command_lower:
                alarm_time += timedelta(days=1)
            
            return alarm_time
        
        return None
    
    def set_alarm(self, alarm_time, command):
        """设置闹钟
        
        Args:
            alarm_time (datetime): 闹钟时间
            command (str): 原始语音指令
        """
        # 生成闹钟ID
        alarm_id = len(self.alarms) + 1
        
        # 创建闹钟字典
        alarm = {
            "id": alarm_id,
            "time": alarm_time,
            "command": command,
            "triggered": False
        }
        
        # 添加到闹钟列表
        self.alarms.append(alarm)
        
        # 发布设置闹钟成功的消息
        reminder_msg = String()
        reminder_msg.data = f"闹钟设置成功！时间: {alarm_time.strftime('%Y-%m-%d %H:%M')}"
        self.reminder_publisher.publish(reminder_msg)
        self.logger.info(f"闹钟设置成功！时间: {alarm_time.strftime('%Y-%m-%d %H:%M')}")
        
    def add_medication_reminder(self, time, medicine_name):
        """添加用药提醒"""
        reminder = {
            "id": len(self.medication_reminders) + 1,
            "time": time,
            "medicine": medicine_name,
            "triggered": False
        }
        self.medication_reminders.append(reminder)
        self.logger.info(f"添加用药提醒: {time} - {medicine_name}")
        
        msg = String()
        msg.data = f"已设置用药提醒: {time.strftime('%H:%M')} 服用 {medicine_name}"
        self.reminder_publisher.publish(msg)

    def trigger_sos(self, reason="未知原因"):
        """触发紧急求助 (SOS)"""
        self.logger.warning(f"触发SOS！原因: {reason}")
        
        # 发布SOS消息
        msg = String()
        msg.data = f"SOS! 原因: {reason}, 联系人: {self.config['emergency_contact']}"
        self.sos_publisher.publish(msg)
        self.reminder_publisher.publish(msg) # 同时在提醒话题发布以便语音播报
        
        # TODO: 这里可以添加发送短信、拨打电话或推送到云端的逻辑

    def check_alarms(self):
        """检查是否有闹钟或提醒需要触发"""
        if self.status != "运行中":
            return
        
        try:
            # 获取当前时间
            now = datetime.now()
            
            # 1. 检查闹钟
            for alarm in self.alarms:
                if not alarm["triggered"] and now >= alarm["time"]:
                    self.trigger_alarm(alarm)
                    alarm["triggered"] = True
            
            # 2. 检查用药提醒
            for reminder in self.medication_reminders:
                if not reminder["triggered"] and now >= reminder["time"]:
                    self.trigger_medication_reminder(reminder)
                    reminder["triggered"] = True
                    
        except Exception as e:
            self.logger.error(f"检查闹钟/提醒时发生错误: {str(e)}")
    
    def trigger_alarm(self, alarm):
        """触发闹钟
        
        Args:
            alarm (dict): 闹钟字典
        """
        # 发布闹钟触发消息
        reminder_msg = String()
        reminder_msg.data = f"闹钟响了！设置时间: {alarm['time'].strftime('%Y-%m-%d %H:%M')}"
        self.reminder_publisher.publish(reminder_msg)
        self.logger.info(f"闹钟触发！时间: {alarm['time'].strftime('%Y-%m-%d %H:%M')}")

    def trigger_medication_reminder(self, reminder):
        """触发用药提醒"""
        msg = String()
        msg.data = f"到了服药时间！请服用: {reminder['medicine']}"
        self.reminder_publisher.publish(msg)
        self.logger.info(f"触发用药提醒: {reminder}")

    def check_schedule(self):
        """检查日程"""
        if self.status != "运行中":
            self.logger.error(f"{self.name}未运行，无法检查日程")
            return
        
        try:
            # 检查日程
            self.logger.info("检查日程...")
            # TODO: 实现实际的日程检查逻辑
            
            # 检查闹钟 (timer每分钟触发一次，但clock_callback也每秒触发check_alarms，双重保险)
            self.check_alarms()
            
        except Exception as e:
            self.logger.error(f"检查日程时发生错误: {str(e)}")
    
    def add_schedule(self, schedule):
        """添加日程"""
        if self.status != "运行中":
            self.logger.error(f"{self.name}未运行，无法添加日程")
            return False
        
        try:
            # TODO: 实现添加日程逻辑
            self.logger.info(f"添加日程: {schedule}")
            return True
            
        except Exception as e:
            self.logger.error(f"添加日程失败: {str(e)}")
            return False
    
    def get_alarms(self):
        """获取当前的闹钟列表"""
        return self.alarms
    
    def get_status(self):
        """获取模块状态"""
        return {
            "name": self.name,
            "status": self.status,
            "config": self.config,
            "alarms_count": len(self.alarms),
            "medication_reminders": len(self.medication_reminders)
        }
    
    def stop_all_alerts(self):
        """停止所有正在响铃的提醒"""
        self.logger.info("收到停止闹钟指令，停止所有响铃...")
        # TODO: 实际停止播放音频或蜂鸣器的逻辑
        stop_msg = String()
        stop_msg.data = "STOP_ALARM"
        self.reminder_publisher.publish(stop_msg)
    
    def stop(self):
        """停止生活辅助模块"""
        self.logger.info(f"正在停止{self.name}...")
        
        try:
            # 停止日程管理
            self._stop_schedule_manager()
            
            # 停止健康监测
            self._stop_health_monitoring()
            
            # 停止用药提醒
            self._stop_medication_reminder()
            
            # 清空闹钟列表
            self.alarms.clear()
            self.medication_reminders.clear()
            
            self.status = "已停止"
            self.logger.info(f"{self.name}已停止")
            
        except Exception as e:
            self.logger.error(f"停止{self.name}时发生错误: {str(e)}")
    
    def _stop_schedule_manager(self):
        """停止日程管理"""
        self.logger.info("停止日程管理...")
        # TODO: 实现日程管理停止逻辑
        
    def _stop_health_monitoring(self):
        """停止健康监测"""
        self.logger.info("停止健康监测...")
        # TODO: 实现健康监测停止逻辑
        
    def _stop_medication_reminder(self):
        """停止用药提醒"""
        self.logger.info("停止用药提醒...")
        # TODO: 实现用药提醒停止逻辑

def main(args=None):
    rclpy.init(args=args)
    schedule_reminder_node = ScheduleReminderNode()
    rclpy.spin(schedule_reminder_node)
    schedule_reminder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
