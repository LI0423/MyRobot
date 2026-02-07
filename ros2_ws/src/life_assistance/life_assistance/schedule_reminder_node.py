#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from voice_msgs.msg import VoiceCommand
from voice_msgs.srv import ToolCall
from life_assistance.msg import Reminder

# 添加项目根目录到Python路径
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../../../..')

# 导入工具模块
from utils.logger import logger_manager

# 导入其他必要的模块
import re
import uuid
import json
from datetime import datetime, timedelta
from threading import Lock
from calendar import monthrange

from memory.memory_manager import MemoryManager
from memory.models.reminder import ReminderRecord

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
            "emergency_contact": "13800000000", # 示例紧急联系人
            "confirm_timeout_minutes": 30,
            "default_snooze_minutes": 10
        }
        
        # 发布提醒话题
        self.reminder_publisher = self.create_publisher(
            Reminder, 'schedule_reminder', 10)
        
        # 发布紧急求助话题
        self.sos_publisher = self.create_publisher(
            Reminder, 'emergency_sos', 10)
        
        # 订阅 voice_command 话题
        self.voice_command_subscriber = self.create_subscription(
            VoiceCommand, 'voice_command', self.voice_command_callback, 10)

        # 语音助手/桥接层工具调用服务
        self.tool_call_service = self.create_service(
            ToolCall, 'life_assistance/tool_call', self.tool_call_callback
        )
        
        # 定时器，每秒检查一次提醒
        self.timer = self.create_timer(1.0, self._tick)
        
        # 初始化闹钟列表
        self.alarms = []
        
        # 初始化用药提醒列表
        self.medication_reminders = []

        # 记忆库（持久化提醒）
        project_root = os.path.dirname(os.path.abspath(__file__)) + '/../../../../..'
        db_path = os.path.join(project_root, "data", "life_assistance.db")
        self.user_id = "default"
        self.memory_manager = MemoryManager(storage_backend='local', db_path=db_path)

        # 初始化当前时间
        self.current_time = datetime.now()
        self._check_lock = Lock()
        self._assistant_request_cache = {}
        
        # 初始化模块
        self.initialize()
    
    def _tick(self):
        """定时器 tick，统一检查提醒"""
        self.current_time = self._now()
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
        # 加载持久化提醒
        self._load_persisted_reminders()
        
    def _init_health_monitoring(self):
        """初始化健康监测"""
        self.logger.info("初始化健康监测...")
        # TODO: 实现健康监测初始化逻辑
        
    def _init_medication_reminder(self):
        """初始化用药提醒"""
        self.logger.info("初始化用药提醒...")
        # 示例：添加一个默认的用药提醒（如果列表为空）
        if not self.medication_reminders:
            now = self._now()
            morning_time = now.replace(hour=8, minute=0, second=0, microsecond=0)
            if morning_time < now:
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

        # 处理工具调用 JSON（语音助手返回 method + params）
        tool_result = self._handle_tool_call_json(command)
        if tool_result is not None:
            if not tool_result.get("success", False):
                self.logger.warning(f"工具调用失败: {tool_result}")
            return

        # 处理确认语义（支持更多自然表达）
        if self._handle_confirmation_intent(command):
            return

        # 处理取消语义（支持最近/全部/药名）
        if self._handle_cancellation_intent(command):
            return

        # 处理延后语义（Snooze）
        if self._handle_snooze_intent(command):
            return

        # 处理结构化指令 (适配 XiaoZhi 节点发来的 params)
        if command.startswith("set_medication:"):
            # set_medication:time:medicine_name
            try:
                parts = command.split(":")
                time_str = parts[1] # HHMM
                medicine = parts[2] if len(parts) > 2 else "药物"
                
                now = self.current_time or self._now()
                hour = int(time_str[:2])
                minute = int(time_str[2:])
                if not self._is_valid_time(hour, minute):
                    self.logger.warning(f"无效用药时间: {time_str}")
                    return
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
            repeat_rule = self.parse_repeat_rule(command)
            self.set_alarm(alarm_time, command, repeat_rule=repeat_rule)
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
        now = self.current_time or self._now()
        
        # 匹配 "X点Y分" 格式
        pattern1 = r'(\d+)点(\d+)?分?'
        match1 = re.search(pattern1, command_lower)
        if match1:
            hour = int(match1.group(1))
            minute = int(match1.group(2)) if match1.group(2) else 0
            if not self._is_valid_time(hour, minute):
                self.logger.warning(f"语音时间超出范围: {hour}:{minute}")
                return None
            
            # 检查是否有 "明天"、"后天" 关键词
            if "明天" in command_lower:
                # 明天的时间
                target_date = now + timedelta(days=1)
            elif "后天" in command_lower:
                # 后天的时间
                target_date = now + timedelta(days=2)
            else:
                # 今天的时间
                target_date = now
            
            # 构建闹钟时间
            alarm_time = self._build_time(target_date, hour, minute)
            if alarm_time is None:
                return None
            
            # 如果今天的时间已经过了，则设置为明天的时间
            if alarm_time < now and "明天" not in command_lower and "后天" not in command_lower:
                alarm_time += timedelta(days=1)
            
            return alarm_time
        
        # 匹配 "X:Y" 格式
        pattern2 = r'(\d+):(\d+)'
        match2 = re.search(pattern2, command_lower)
        if match2:
            hour = int(match2.group(1))
            minute = int(match2.group(2))
            if not self._is_valid_time(hour, minute):
                self.logger.warning(f"语音时间超出范围: {hour}:{minute}")
                return None
            
            # 检查是否有 "明天"、"后天" 关键词
            if "明天" in command_lower:
                # 明天的时间
                target_date = now + timedelta(days=1)
            elif "后天" in command_lower:
                # 后天的时间
                target_date = now + timedelta(days=2)
            else:
                # 今天的时间
                target_date = now
            
            # 构建闹钟时间
            alarm_time = self._build_time(target_date, hour, minute)
            if alarm_time is None:
                return None
            
            # 如果今天的时间已经过了，则设置为明天的时间
            if alarm_time < now and "明天" not in command_lower and "后天" not in command_lower:
                alarm_time += timedelta(days=1)
            
            return alarm_time
        
        return None
    
    def set_alarm(self, alarm_time, command, repeat_rule=None):
        """设置闹钟
        
        Args:
            alarm_time (datetime): 闹钟时间
            command (str): 原始语音指令
        """
        if not isinstance(alarm_time, datetime):
            raise ValueError("alarm_time 必须是 datetime 类型")
        if not isinstance(command, str) or not command.strip():
            raise ValueError("command 不能为空")
        if repeat_rule and repeat_rule not in ("daily", "weekly", "monthly"):
            raise ValueError("repeat_rule 仅支持 daily/weekly/monthly")

        # 生成闹钟ID
        alarm_id = str(uuid.uuid4())
        
        # 创建闹钟字典
        alarm = {
            "id": alarm_id,
            "time": alarm_time,
            "command": command,
            "triggered": False,
            "status": "pending",
            "repeat_rule": repeat_rule,
            "payload": {"command": command}
        }
        
        # 添加到闹钟列表
        self.alarms.append(alarm)
        self._persist_reminder(
            ReminderRecord(
                reminder_id=alarm_id,
                user_id=self.user_id,
                kind="alarm",
                due_time=alarm_time,
                repeat_rule=repeat_rule,
                status="pending",
                payload={"command": command}
            )
        )
        
        # 发布设置闹钟成功的消息
        reminder_msg = self._build_reminder_msg(
            reminder_id=alarm_id,
            reminder_type="alarm_set",
            data=f"闹钟设置成功！时间: {alarm_time.strftime('%Y-%m-%d %H:%M')}",
            status="pending",
            due_time=alarm_time,
            repeat_rule=repeat_rule,
            command=command
        )
        self.reminder_publisher.publish(reminder_msg)
        self.logger.info(f"闹钟设置成功！时间: {alarm_time.strftime('%Y-%m-%d %H:%M')}")
        return alarm

    def handle_tool_call(self, method_name, params):
        """
        供语音助手调用的统一工具入口

        Args:
            method_name (str): 方法名，如 set_alarm
            params (dict): 参数字典

        Returns:
            dict: 统一响应 {"success": bool, ...}
        """
        params = params or {}
        if method_name == "set_alarm":
            return self.assistant_set_alarm(**params)
        return {"success": False, "error_code": "UNSUPPORTED_METHOD", "message": f"不支持的方法: {method_name}"}

    def tool_call_callback(self, request, response):
        """ROS2 service: 执行工具调用"""
        try:
            params = {}
            if request.params_json:
                params = json.loads(request.params_json)
                if not isinstance(params, dict):
                    raise ValueError("params_json 必须是 JSON 对象")
            result = self.handle_tool_call(request.method, params)
            response.success = bool(result.get("success", False))
            response.result_json = json.dumps(result, ensure_ascii=False)
            response.error_code = result.get("error_code", "")
            response.error_message = result.get("message", "")
            return response
        except Exception as e:
            response.success = False
            response.result_json = ""
            response.error_code = "SERVICE_ERROR"
            response.error_message = str(e)
            self.logger.error(f"tool_call_callback 失败: {e}")
            return response

    def assistant_set_alarm(
        self,
        due_time,
        command,
        repeat_rule=None,
        source="assistant",
        idempotency_key=None
    ):
        """
        语音助手调用：设置闹钟

        Args:
            due_time (str): ISO 时间字符串，如 2026-02-08T08:30:00
            command (str): 原始语义文本
            repeat_rule (str, optional): daily/weekly/monthly
            source (str, optional): 调用来源标识
            idempotency_key (str, optional): 幂等键，防重复下发

        Returns:
            dict: 结果信息
        """
        try:
            if not isinstance(due_time, str) or not due_time.strip():
                return {"success": False, "error_code": "INVALID_PARAM", "message": "due_time 必填且必须是 ISO 字符串"}
            if not isinstance(command, str) or not command.strip():
                return {"success": False, "error_code": "INVALID_PARAM", "message": "command 必填且不能为空"}
            if repeat_rule and repeat_rule not in ("daily", "weekly", "monthly"):
                return {"success": False, "error_code": "INVALID_PARAM", "message": "repeat_rule 仅支持 daily/weekly/monthly"}

            if idempotency_key and idempotency_key in self._assistant_request_cache:
                cached = self._assistant_request_cache[idempotency_key]
                return {
                    "success": True,
                    "alarm_id": cached["alarm_id"],
                    "status": "pending",
                    "due_time": cached["due_time"],
                    "repeat_rule": cached.get("repeat_rule"),
                    "idempotent": True,
                }

            alarm_time = self._parse_iso_time(due_time)
            if alarm_time is None:
                return {"success": False, "error_code": "INVALID_TIME", "message": "due_time 解析失败"}

            now = self.current_time or self._now()
            if alarm_time <= now:
                return {"success": False, "error_code": "TIME_IN_PAST", "message": "due_time 不能早于当前时间"}

            alarm = self.set_alarm(alarm_time, command, repeat_rule=repeat_rule)
            result = {
                "success": True,
                "alarm_id": alarm["id"],
                "status": alarm["status"],
                "due_time": alarm["time"].isoformat(),
                "repeat_rule": alarm.get("repeat_rule"),
                "source": source,
            }
            if idempotency_key:
                self._assistant_request_cache[idempotency_key] = {
                    "alarm_id": alarm["id"],
                    "due_time": alarm["time"].isoformat(),
                    "repeat_rule": alarm.get("repeat_rule"),
                }
            return result
        except Exception as e:
            self.logger.error(f"assistant_set_alarm 调用失败: {e}")
            return {"success": False, "error_code": "INTERNAL_ERROR", "message": str(e)}
        
    def add_medication_reminder(self, time, medicine_name):
        """添加用药提醒"""
        reminder = {
            "id": str(uuid.uuid4()),
            "time": time,
            "medicine": medicine_name,
            "triggered": False,
            "status": "pending",
            "repeat_rule": None,
            "payload": {"medicine": medicine_name}
        }
        self.medication_reminders.append(reminder)
        self.logger.info(f"添加用药提醒: {time} - {medicine_name}")

        self._persist_reminder(
            ReminderRecord(
                reminder_id=reminder["id"],
                user_id=self.user_id,
                kind="medication",
                due_time=time,
                status="pending",
                payload={"medicine": medicine_name}
            )
        )
        
        msg = self._build_reminder_msg(
            reminder_id=reminder["id"],
            reminder_type="medication_set",
            data=f"已设置用药提醒: {time.strftime('%H:%M')} 服用 {medicine_name}",
            status="pending",
            due_time=time,
            medicine=medicine_name
        )
        self.reminder_publisher.publish(msg)

    def trigger_sos(self, reason="未知原因"):
        """触发紧急求助 (SOS)"""
        self.logger.warning(f"触发SOS！原因: {reason}")
        
        # 发布SOS消息
        msg = self._build_reminder_msg(
            reminder_id=str(uuid.uuid4()),
            reminder_type="sos",
            data=f"SOS! 原因: {reason}, 联系人: {self.config['emergency_contact']}",
            status="triggered",
            reason=reason,
            contact=self.config["emergency_contact"]
        )
        self.sos_publisher.publish(msg)
        self.reminder_publisher.publish(msg) # 同时在提醒话题发布以便语音播报
        
        # TODO: 这里可以添加发送短信、拨打电话或推送到云端的逻辑

    def check_alarms(self):
        """检查是否有闹钟或提醒需要触发"""
        if self.status != "运行中":
            return
        if not self._check_lock.acquire(blocking=False):
            return
        try:
            # 获取当前时间
            now = self.current_time or self._now()

            # 1. 检查闹钟
            for alarm in list(self.alarms):
                if not alarm["triggered"] and now >= alarm["time"]:
                    self.trigger_alarm(alarm)
                    alarm["triggered"] = True
                    alarm["status"] = "triggered"
                    self._set_confirm_deadline(alarm, now)
                    self._update_reminder_fields(alarm["id"], status="triggered", payload=alarm.get("payload", {}))
                    self._schedule_next_repeat(alarm)

            # 2. 检查用药提醒
            for reminder in list(self.medication_reminders):
                if not reminder["triggered"] and now >= reminder["time"]:
                    self.trigger_medication_reminder(reminder)
                    reminder["triggered"] = True
                    reminder["status"] = "triggered"
                    self._set_confirm_deadline(reminder, now)
                    self._update_reminder_fields(reminder["id"], status="triggered", payload=reminder.get("payload", {}))
                    self._schedule_next_repeat(reminder)

            # 3. 检查超时未确认
            self._check_confirm_timeouts(now)

        except Exception as e:
            self.logger.error(f"检查闹钟/提醒时发生错误: {str(e)}")
        finally:
            self._check_lock.release()
    
    def trigger_alarm(self, alarm):
        """触发闹钟
        
        Args:
            alarm (dict): 闹钟字典
        """
        # 发布闹钟触发消息
        reminder_msg = self._build_reminder_msg(
            reminder_id=alarm["id"],
            reminder_type="alarm_triggered",
            data=f"闹钟响了！设置时间: {alarm['time'].strftime('%Y-%m-%d %H:%M')}",
            status="triggered",
            due_time=alarm["time"],
            repeat_rule=alarm.get("repeat_rule"),
            command=alarm.get("command", "")
        )
        self.reminder_publisher.publish(reminder_msg)
        self.logger.info(f"闹钟触发！时间: {alarm['time'].strftime('%Y-%m-%d %H:%M')}")

    def trigger_medication_reminder(self, reminder):
        """触发用药提醒"""
        msg = self._build_reminder_msg(
            reminder_id=reminder["id"],
            reminder_type="medication_triggered",
            data=f"到了服药时间！请服用: {reminder['medicine']}",
            status="triggered",
            due_time=reminder["time"],
            repeat_rule=reminder.get("repeat_rule"),
            medicine=reminder.get("medicine", "")
        )
        self.reminder_publisher.publish(msg)
        self.logger.info(f"触发用药提醒: {reminder}")

    def confirm_reminder(self, reminder_id, source="manual"):
        """确认提醒"""
        target = None
        for alarm in self.alarms:
            if alarm["id"] == reminder_id:
                target = alarm
                break
        if not target:
            for med in self.medication_reminders:
                if med["id"] == reminder_id:
                    target = med
                    break

        if not target:
            self.logger.warning(f"未找到提醒ID: {reminder_id}")
            return

        target["status"] = "confirmed"
        self._update_reminder_fields(reminder_id, status="confirmed", payload=target.get("payload", {}))

        msg = self._build_reminder_msg(
            reminder_id=reminder_id,
            reminder_type="reminder_confirmed",
            data=f"提醒已确认（来源: {source}）",
            status="confirmed",
            due_time=target.get("time"),
            repeat_rule=target.get("repeat_rule")
        )
        self.reminder_publisher.publish(msg)

    def confirm_latest(self, kind="medication", source="manual"):
        """确认最近触发的提醒"""
        candidates = []
        if kind == "alarm":
            candidates = [a for a in self.alarms if a.get("status") == "triggered"]
        elif kind == "medication":
            candidates = [m for m in self.medication_reminders if m.get("status") == "triggered"]
        else:
            candidates = [a for a in self.alarms if a.get("status") == "triggered"] + \
                         [m for m in self.medication_reminders if m.get("status") == "triggered"]

        if not candidates:
            self.logger.info("没有可确认的提醒")
            return

        candidates.sort(key=lambda x: x.get("time"))
        self.confirm_reminder(candidates[-1]["id"], source=source)

    def confirm_all_triggered(self, kind=None, source="manual"):
        """确认全部已触发提醒"""
        if kind == "alarm":
            candidates = [a for a in self.alarms if a.get("status") == "triggered"]
        elif kind == "medication":
            candidates = [m for m in self.medication_reminders if m.get("status") == "triggered"]
        else:
            candidates = [a for a in self.alarms if a.get("status") == "triggered"] + \
                         [m for m in self.medication_reminders if m.get("status") == "triggered"]
        for item in candidates:
            self.confirm_reminder(item["id"], source=source)

    def cancel_reminder(self, reminder_id, source="manual"):
        """取消提醒"""
        removed = False
        for idx, alarm in enumerate(list(self.alarms)):
            if alarm["id"] == reminder_id:
                del self.alarms[idx]
                removed = True
                break
        if not removed:
            for idx, med in enumerate(list(self.medication_reminders)):
                if med["id"] == reminder_id:
                    del self.medication_reminders[idx]
                    removed = True
                    break
        if not removed:
            self.logger.warning(f"未找到提醒ID: {reminder_id}")
            return
        self._update_reminder_fields(reminder_id, status="canceled")
        msg = self._build_reminder_msg(
            reminder_id=reminder_id,
            reminder_type="reminder_canceled",
            data=f"提醒已取消（来源: {source}）",
            status="canceled"
        )
        self.reminder_publisher.publish(msg)

    def cancel_latest(self, source="manual"):
        """取消最近一个提醒"""
        candidates = self.alarms + self.medication_reminders
        if not candidates:
            self.logger.info("没有可取消的提醒")
            return
        candidates.sort(key=lambda x: x.get("time"))
        self.cancel_reminder(candidates[-1]["id"], source=source)

    def cancel_all(self, kind=None, source="manual"):
        """取消全部提醒"""
        if kind == "alarm":
            candidates = [a["id"] for a in self.alarms]
        elif kind == "medication":
            candidates = [m["id"] for m in self.medication_reminders]
        else:
            candidates = [a["id"] for a in self.alarms] + [m["id"] for m in self.medication_reminders]
        for reminder_id in candidates:
            self.cancel_reminder(reminder_id, source=source)

    def snooze_latest(self, minutes=None, source="manual"):
        """延后最近触发的提醒"""
        minutes = minutes or int(self.config.get("default_snooze_minutes", 10))
        candidates = [a for a in self.alarms if a.get("status") == "triggered"] + \
                     [m for m in self.medication_reminders if m.get("status") == "triggered"]
        if not candidates:
            self.logger.info("没有可延后的提醒")
            return
        candidates.sort(key=lambda x: x.get("time"))
        target = candidates[-1]
        new_time = (self.current_time or self._now()) + timedelta(minutes=minutes)
        target["time"] = new_time
        target["status"] = "pending"
        target["triggered"] = False
        self._update_reminder_fields(target["id"], status="pending", payload=target.get("payload", {}))
        msg = self._build_reminder_msg(
            reminder_id=target["id"],
            reminder_type="reminder_snoozed",
            data=f"提醒已延后 {minutes} 分钟（来源: {source}）",
            status="pending",
            due_time=new_time,
            repeat_rule=target.get("repeat_rule"),
            medicine=target.get("medicine", ""),
            command=target.get("command", "")
        )
        self.reminder_publisher.publish(msg)

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

    def _load_persisted_reminders(self):
        """从记忆库加载未完成的提醒"""
        try:
            reminders = self.memory_manager.get_reminders(self.user_id)
        except Exception as e:
            self.logger.error(f"加载提醒失败: {e}")
            return

        self.alarms = []
        self.medication_reminders = []
        for record in reminders:
            if record.status in ("confirmed", "expired", "canceled"):
                continue
            payload = record.payload if isinstance(record.payload, dict) else {}
            if record.kind == "alarm":
                if self._should_expire_record(record):
                    record.update(status="expired")
                    self.memory_manager.update_reminder(record)
                    continue
                self.alarms.append({
                    "id": record.reminder_id,
                    "time": record.due_time,
                    "command": payload.get("command", ""),
                    "payload": payload,
                    "triggered": record.status == "triggered",
                    "status": record.status,
                    "repeat_rule": record.repeat_rule
                })
            elif record.kind == "medication":
                if self._should_expire_record(record):
                    record.update(status="expired")
                    self.memory_manager.update_reminder(record)
                    continue
                self.medication_reminders.append({
                    "id": record.reminder_id,
                    "time": record.due_time,
                    "medicine": payload.get("medicine", ""),
                    "payload": payload,
                    "triggered": record.status == "triggered",
                    "status": record.status,
                    "repeat_rule": record.repeat_rule
                })

        self.logger.info(
            f"加载提醒完成: 闹钟={len(self.alarms)} 用药={len(self.medication_reminders)}"
        )

    def _persist_reminder(self, reminder_record):
        """保存提醒到记忆库"""
        try:
            self.memory_manager.add_reminder(reminder_record)
        except Exception as e:
            self.logger.error(f"保存提醒失败: {e}")

    def _update_reminder_fields(self, reminder_id, status=None, payload=None):
        """更新提醒状态/负载"""
        try:
            reminders = self.memory_manager.get_reminders(self.user_id)
            for record in reminders:
                if record.reminder_id == reminder_id:
                    if status:
                        record.update(status=status)
                    if payload is not None:
                        merged_payload = record.payload.copy() if isinstance(record.payload, dict) else {}
                        merged_payload.update(payload)
                        record.update(payload=merged_payload)
                    self.memory_manager.update_reminder(record)
                    break
        except Exception as e:
            self.logger.error(f"更新提醒状态失败: {e}")

    def _set_confirm_deadline(self, reminder, now):
        """设置确认截止时间"""
        timeout_minutes = int(self.config.get("confirm_timeout_minutes", 30))
        deadline = now + timedelta(minutes=timeout_minutes)
        reminder["payload"] = reminder.get("payload", {})
        reminder["payload"]["confirm_deadline"] = deadline.isoformat()

    def _check_confirm_timeouts(self, now):
        """检查超时未确认提醒"""
        def _expired(rem):
            payload = rem.get("payload", {})
            deadline_str = payload.get("confirm_deadline")
            if not deadline_str:
                return False
            try:
                deadline = datetime.fromisoformat(deadline_str)
            except Exception:
                return False
            return now >= deadline

        for alarm in self.alarms:
            if alarm.get("status") == "triggered" and _expired(alarm):
                alarm["status"] = "expired"
                self._update_reminder_fields(alarm["id"], status="expired", payload=alarm.get("payload", {}))
                msg = self._build_reminder_msg(
                    reminder_id=alarm["id"],
                    reminder_type="reminder_expired",
                    data="闹钟提醒超时未确认",
                    status="expired",
                    due_time=alarm.get("time"),
                    repeat_rule=alarm.get("repeat_rule")
                )
                self.reminder_publisher.publish(msg)

        for reminder in self.medication_reminders:
            if reminder.get("status") == "triggered" and _expired(reminder):
                reminder["status"] = "expired"
                self._update_reminder_fields(reminder["id"], status="expired", payload=reminder.get("payload", {}))
                msg = self._build_reminder_msg(
                    reminder_id=reminder["id"],
                    reminder_type="reminder_expired",
                    data="用药提醒超时未确认",
                    status="expired",
                    due_time=reminder.get("time"),
                    repeat_rule=reminder.get("repeat_rule"),
                    medicine=reminder.get("medicine", "")
                )
                self.reminder_publisher.publish(msg)

    def _schedule_next_repeat(self, reminder):
        """根据重复规则生成下一次提醒"""
        rule = reminder.get("repeat_rule")
        if not rule:
            return
        next_time = None
        if rule == "daily":
            next_time = reminder["time"] + timedelta(days=1)
        elif rule == "weekly":
            next_time = reminder["time"] + timedelta(days=7)
        elif rule == "monthly":
            next_time = self._add_months(reminder["time"], 1)

        if not next_time:
            return

        if reminder.get("medicine"):
            self.add_medication_reminder(next_time, reminder.get("medicine"))
        else:
            self.set_alarm(next_time, reminder.get("command", ""), repeat_rule=rule)

    def _should_expire_record(self, record):
        """启动时判断是否过期"""
        if record.status != "triggered":
            return False
        payload = record.payload if isinstance(record.payload, dict) else {}
        deadline = payload.get("confirm_deadline")
        if not deadline:
            return False
        try:
            deadline_dt = datetime.fromisoformat(deadline)
        except Exception:
            return False
        now = self._now()
        return now >= deadline_dt

    def _parse_snooze_minutes(self, command):
        """解析延后分钟数"""
        pattern = r'(\d+)\s*分钟'
        match = re.search(pattern, command)
        if match:
            try:
                return int(match.group(1))
            except Exception:
                pass
        return int(self.config.get("default_snooze_minutes", 10))

    def _handle_tool_call_json(self, command):
        """
        识别并处理语音助手返回的工具调用 JSON
        JSON 格式: {"method":"set_alarm","params":{...}}
        """
        if not isinstance(command, str):
            return None
        text = command.strip()
        if not (text.startswith("{") and text.endswith("}")):
            return None
        try:
            data = json.loads(text)
        except Exception:
            return None
        if not isinstance(data, dict):
            return None
        method_name = data.get("method")
        params = data.get("params", {})
        if not method_name:
            return None
        return self.handle_tool_call(method_name, params)

    def _parse_iso_time(self, due_time):
        """解析 ISO 时间字符串并归一为本地 naive datetime"""
        try:
            dt = datetime.fromisoformat(due_time)
        except Exception:
            return None
        if dt.tzinfo is not None:
            dt = dt.astimezone().replace(tzinfo=None)
        return dt

    def _is_valid_time(self, hour, minute):
        """校验小时和分钟范围"""
        return 0 <= hour <= 23 and 0 <= minute <= 59

    def _build_time(self, base_date, hour, minute):
        """安全构建目标时间"""
        try:
            return base_date.replace(hour=hour, minute=minute, second=0, microsecond=0)
        except ValueError:
            self.logger.warning(f"构建时间失败: {hour}:{minute}")
            return None

    def _add_months(self, dt, months):
        """按自然月增加月份"""
        month = dt.month - 1 + months
        year = dt.year + month // 12
        month = month % 12 + 1
        day = min(dt.day, monthrange(year, month)[1])
        return dt.replace(year=year, month=month, day=day)

    def _handle_confirmation_intent(self, command):
        """识别并处理确认意图"""
        cmd = command.strip()

        if cmd.startswith("confirm_reminder:"):
            reminder_id = cmd.split(":", 1)[1].strip()
            if reminder_id:
                self.confirm_reminder(reminder_id, source="voice")
                return True

        if cmd.startswith("confirm_medication"):
            self.confirm_latest(kind="medication", source="voice")
            return True

        confirm_all_phrases = ["全部确认", "都确认", "全部已处理", "都处理完了", "全部已完成"]
        if any(p in cmd for p in confirm_all_phrases):
            if "闹钟" in cmd:
                self.confirm_all_triggered(kind="alarm", source="voice")
            elif "药" in cmd or "用药" in cmd:
                self.confirm_all_triggered(kind="medication", source="voice")
            else:
                self.confirm_all_triggered(kind=None, source="voice")
            return True

        med_confirm_phrases = [
            "已吃药", "吃过药", "我吃药了", "服药了", "药已经吃了", "药吃完了", "用药完成"
        ]
        if any(p in cmd for p in med_confirm_phrases):
            self.confirm_latest(kind="medication", source="voice")
            return True

        generic_confirm_phrases = ["确认提醒", "确认闹钟", "知道了", "我知道了", "收到", "好的提醒"]
        if any(p in cmd for p in generic_confirm_phrases):
            if "闹钟" in cmd:
                self.confirm_latest(kind="alarm", source="voice")
            elif "药" in cmd or "用药" in cmd:
                self.confirm_latest(kind="medication", source="voice")
            else:
                self.confirm_latest(kind=None, source="voice")
            return True

        return False

    def _handle_cancellation_intent(self, command):
        """识别并处理取消意图"""
        cmd = command.strip()

        if cmd.startswith("cancel_reminder:"):
            reminder_id = cmd.split(":", 1)[1].strip()
            if reminder_id:
                self.cancel_reminder(reminder_id, source="voice")
                return True

        cancel_all_phrases = ["取消所有", "删除所有", "全部取消", "全部删除", "清空提醒"]
        if any(p in cmd for p in cancel_all_phrases):
            if "闹钟" in cmd:
                self.cancel_all(kind="alarm", source="voice")
            elif "药" in cmd or "用药" in cmd:
                self.cancel_all(kind="medication", source="voice")
            else:
                self.cancel_all(kind=None, source="voice")
            return True

        cancel_latest_phrases = ["取消提醒", "取消闹钟", "删除闹钟", "取消用药提醒", "不用提醒了", "撤销提醒"]
        if any(p in cmd for p in cancel_latest_phrases):
            if "闹钟" in cmd:
                self.cancel_latest(source="voice")
            elif "药" in cmd or "用药" in cmd:
                self._cancel_latest_medication(source="voice")
            else:
                self.cancel_latest(source="voice")
            return True

        # 按药名取消（例如：取消降压药提醒）
        if "取消" in cmd and ("药" in cmd or "用药" in cmd):
            for reminder in sorted(self.medication_reminders, key=lambda x: x.get("time"), reverse=True):
                medicine = reminder.get("medicine", "")
                if medicine and medicine in cmd:
                    self.cancel_reminder(reminder["id"], source="voice")
                    return True

        return False

    def _cancel_latest_medication(self, source="manual"):
        """取消最近一个用药提醒"""
        candidates = [m for m in self.medication_reminders]
        if not candidates:
            self.logger.info("没有可取消的用药提醒")
            return
        candidates.sort(key=lambda x: x.get("time"))
        self.cancel_reminder(candidates[-1]["id"], source=source)

    def _handle_snooze_intent(self, command):
        """识别并处理延后意图"""
        cmd = command.strip()
        if not any(phrase in cmd for phrase in ["稍后", "延后", "过一会", "等会再提醒", "一会再提醒"]):
            return False
        minutes = self._parse_snooze_minutes(cmd)
        self.snooze_latest(minutes=minutes, source="voice")
        return True

    def parse_repeat_rule(self, command):
        """解析重复规则"""
        if "每天" in command:
            return "daily"
        if "每周" in command:
            return "weekly"
        if "每月" in command:
            return "monthly"
        return None

    def _now(self):
        """获取当前时间（ROS time）"""
        time_msg = self.get_clock().now().to_msg()
        return datetime.fromtimestamp(time_msg.sec + time_msg.nanosec / 1e9)
    
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
        stop_msg = self._build_reminder_msg(
            reminder_id=str(uuid.uuid4()),
            reminder_type="stop_alarm",
            data="STOP_ALARM",
            status="triggered"
        )
        self.reminder_publisher.publish(stop_msg)

    def _build_reminder_msg(
        self,
        reminder_id,
        reminder_type,
        data,
        status="",
        due_time=None,
        repeat_rule="",
        medicine="",
        command="",
        reason="",
        contact=""
    ):
        msg = Reminder()
        msg.id = reminder_id or ""
        msg.type = reminder_type or ""
        msg.data = data or ""
        msg.status = status or ""
        msg.due_time = due_time.isoformat() if due_time else ""
        msg.repeat_rule = repeat_rule or ""
        msg.medicine = medicine or ""
        msg.command = command or ""
        msg.reason = reason or ""
        msg.contact = contact or ""
        return msg
    
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
