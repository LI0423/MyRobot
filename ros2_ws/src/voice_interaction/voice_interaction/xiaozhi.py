import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 尝试导入语音消息类型，如果失败则使用String
try:
    from voice_msgs.msg import VoiceCommand
    VOICE_MSGS_AVAILABLE = True
except ImportError:
    VOICE_MSGS_AVAILABLE = False
    print("Warning: voice_msgs not found, VoiceCommand integration will be limited.")

import threading
import warnings
import urllib3
import logging
import os
import time
import re
from datetime import datetime

from llm_node.mqtt_service import MQTTService
from llm_node.audio_service import AudioService
from dotenv import load_dotenv
from llm_node.enums import MessageType
from llm_node.config import Config

# Get logger
logger = logging.getLogger(__name__)

# 屏蔽警告
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)  # 屏蔽urllib3的不安全请求警告
warnings.filterwarnings("ignore", category=DeprecationWarning)  # 屏蔽弃用警告

# 配置详细日志
logging.basicConfig(
    level=logging.INFO,  # 提高日志级别为INFO，输出更详细的信息
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # 增加日志来源名称
    handlers=[
        logging.FileHandler('xiaozhi_node_vad.log', mode='w'),  # 日志文件
        logging.StreamHandler()  # 同时输出到控制台
    ]
)

# 为特定模块设置更详细的日志级别
logging.getLogger('llm_node.mqtt_service').setLevel(logging.DEBUG)
logging.getLogger('llm_node.audio_service').setLevel(logging.DEBUG)

load_dotenv()  # 加载.env文件中的环境变量

class ALSAErrorSuppressor:
    """ALSA错误输出抑制器，防止音频库错误信息干扰用户界面
    
    注意：此类已在audio_service.py中定义，此处为重复定义
    """

    def __enter__(self):
        """进入上下文管理器，将stderr重定向到/dev/null"""
        self.old_stderr = os.dup(2)  # 保存原始stderr文件描述符
        self.devnull = os.open('/dev/null', os.O_WRONLY)  # 打开/dev/null用于写入
        os.dup2(self.devnull, 2)  # 将stderr重定向到/dev/null
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文管理器，恢复原始stderr"""
        os.dup2(self.old_stderr, 2)  # 恢复原始stderr
        os.close(self.old_stderr)  # 关闭保存的文件描述符
        os.close(self.devnull)  # 关闭/dev/null文件描述符


class XiaoZhiNode(Node):
    """小智节点主类，整合MQTT服务和音频服务，处理各种消息"""
    
    def __init__(self):
        """初始化小智节点"""
        super().__init__('xiaozhi_node')
        logger.info("正在初始化小智节点...")

        # 初始化配置
        self.config = Config()
        
        # 初始化ROS发布者
        self._init_publishers()
        
        # 初始化MQTT服务
        logger.debug("正在初始化MQTT服务...")
        self.mqtt_service = MQTTService(self.config)
        self.mqtt_service.set_message_callback(self._on_mqtt_message)  # 设置MQTT消息回调
        logger.info("MQTT服务初始化完成")
        
        # 初始化音频服务
        logger.debug("正在初始化音频服务...")
        self.audio_service = AudioService(self.config, 
                                         on_listen_start=self._on_listen_start, 
                                         on_listen_stop=self._on_listen_stop,
                                         on_wake_word_detected=self._on_wake_word_detected)
        logger.info("音频服务初始化完成")
        
        # 启动MQTT和音频服务的线程
        logger.debug("正在启动服务线程...")
        threading.Thread(target=self._start_mqtt_and_audio, daemon=True, name="service_start_thread").start()
        
        # 状态变量
        self.tts_state = None  # TTS播放状态
        self.last_printed_text = ""  # 上次打印的文本
        self.session_id = None  # 会话ID
        self.lock = threading.Lock()  # 线程锁
        self.last_listen_stop_time = None  # 上次监听停止时间
        logger.info("小智节点初始化完成")

    def _init_publishers(self):
        """初始化ROS发布者"""
        # 家居控制命令发布者
        self.home_control_pub = self.create_publisher(
            String, 'home_control_command', 10)
        
        # 语音指令发布者 (用于生活辅助等)
        if VOICE_MSGS_AVAILABLE:
            self.voice_command_pub = self.create_publisher(
                VoiceCommand, 'voice_command', 10)
        else:
            self.voice_command_pub = None
            
        logger.info("ROS发布者初始化完成")
        
    def _start_mqtt_and_audio(self):
        """启动MQTT和音频服务"""
        logger.info("正在启动MQTT和音频服务...")
        try:
            # 启动MQTT服务
            logger.debug("正在启动MQTT服务...")
            self.mqtt_service.start()
            logger.info("MQTT服务启动完成")
            
            # 订阅MQTT主题
            logger.debug("正在订阅MQTT主题...")
            result = self.mqtt_service.subscribe()
            logger.info(f"MQTT主题订阅完成，结果: {result}")
        except Exception as e:
            logger.exception('MQTT服务启动失败')
        
        # 启动音频服务
        logger.debug("正在启动音频服务...")
        self.audio_service.start()
        logger.info("音频服务启动完成")

    def _on_mqtt_message(self, topic, message, raw_msg):
        """MQTT消息回调函数
        
        Args:
            topic (str): 消息主题
            message (dict): 解析后的消息内容
            raw_msg: 原始消息对象
        """
        logger.info(f'收到MQTT消息，主题: {topic}，内容: {message}')
        if not message:
            logger.warning('收到空的MQTT消息')
            return
        
        # 获取消息类型
        message_type = message.get('type')
        logger.debug(f'消息类型: {message_type}')
        
        # 处理不同类型的消息
        if message_type == MessageType.HELLO.value:
            logger.info('处理HELLO消息')
            self.handle_hello_message(message)
        elif message_type == MessageType.TTS.value:
            logger.info('处理TTS消息')
            self.handle_tts_message(message)
        elif message_type == MessageType.STT.value:
            logger.info('处理STT消息')
            self.handle_stt_message(message)
        elif message_type == MessageType.LLM.value:
            logger.info('处理LLM消息')
            self.handle_llm_message(message)
        elif message_type == MessageType.GOODBYE.value:
            logger.info('处理GOODBYE消息')
            self.handle_goodbye_message(message)
        elif message_type == MessageType.HEARTBEAT.value:
            logger.debug('收到心跳消息，忽略')
        else:
            logger.warning(f'收到未知类型的消息: {message_type}')
    
    def handle_hello_message(self, message):
        """处理HELLO消息，用于建立会话
        
        Args:
            message (dict): HELLO消息内容
        """
        logger.info(f'处理HELLO消息，当前会话ID: {self.session_id}')
        
        with self.lock:
            if not self.session_id:
                # 保存会话ID和UDP信息
                self.session_id = message.get('session_id', None)
                udp_info = message.get('udp', self.audio_service.udp_info)
                logger.info(f'创建新会话，会话ID: {self.session_id}, UDP信息: {udp_info}')
                self.audio_service.update_udp_info(udp_info)
            else:
                logger.info(f'会话已存在，会话ID: {self.session_id}')
        
        logger.info(f"HELLO消息处理完成, 会话ID: {self.session_id}")
        
        # 重启音频流，确保连接到正确的服务端
        logger.debug('正在重启音频流...')
        self.audio_service.restart_audio_streams()
        logger.info('音频流重启完成')
        
    def handle_tts_message(self, message):
        """处理TTS（文本转语音）消息
        
        Args:
            message (dict): TTS消息内容
        """
        logger.info(f'处理TTS消息: {message}')
        
        # 获取TTS状态和文本
        self.tts_state = message.get('state')
        text = message.get('text', '')
        logger.debug(f'TTS状态: {self.tts_state}, 文本: {text}')
        
        if self.tts_state == 'start':
            # TTS开始播放
            logger.info('TTS开始播放')
        elif self.tts_state == 'sentence_start':
            # 显示AI回复文本
            logger.info(f'显示AI回复: {text}')
            if text and text != self.last_printed_text:
                logger.debug(f'更新上次打印文本: {text}')
                self.last_printed_text = text
        elif self.tts_state == 'stop':
            # TTS停止播放
            logger.info('TTS播放停止')
            self.last_printed_text = ""
        else:
            logger.warning(f'未知的TTS状态: {self.tts_state}')
        logger.info('TTS消息处理完成')
            
    def send_hello_message(self):
        """发送HELLO消息，请求建立会话
        
        构造并发送包含设备信息的HELLO消息到MQTT服务器，用于建立会话
        """
        logger.info("正在发送HELLO消息，请求建立会话...")
        try:
            # 构造HELLO消息
            hello_msg = {
                "type": "hello",
                "version": 3,
                "transport": "udp",
                "audio_params": {
                    "format": "opus",
                    "sample_rate": self.config.sample_rate,
                    "channels": 1,
                    "frame_duration": 60
                },
                "udp": self.audio_service.udp_info,
                "session_id": None  # 初始会话ID为空，由服务器分配
            }
            
            # 通过MQTT发送HELLO消息
            result = self.mqtt_service.publish(hello_msg)
            logger.info(f"HELLO消息发送完成，结果: {result}")
        except Exception as e:
            logger.error(f"发送HELLO消息失败: {str(e)}")
    
    def send_listen_message(self, state):
        """发送LISTEN消息，通知服务器录音状态
        
        Args:
            state (str): 录音状态，"start"或"stop"
        """
        logger.info(f"正在发送LISTEN消息，状态: {state}")
        try:
            # 构造LISTEN消息
            listen_msg = {
                "type": "listen",
                "state": state,
                "session_id": self.session_id
            }
            
            # 通过MQTT发送LISTEN消息
            result = self.mqtt_service.publish(listen_msg)
            logger.info(f"LISTEN消息发送完成，结果: {result}")
        except Exception as e:
            logger.error(f"发送LISTEN消息失败: {str(e)}")
    
    def handle_stt_message(self, message):
        """处理STT（语音转文本）消息
        
        Args:
            message (dict): STT消息内容
        """
        logger.info(f'处理STT消息: {message}')
        stt_text = message.get('text', '')
        if stt_text:
            logger.info(f'STT识别结果: {stt_text}')
            print(f"👂 用户: {stt_text}")
        else:
            logger.warning('STT结果为空')
        logger.info('STT消息处理完成')

    def handle_llm_message(self, message):
        """处理LLM（大语言模型）消息
        
        Args:
            message (dict): LLM消息内容
        """
        logger.info(f'处理LLM消息: {message}')
        llm_text = message.get('text', '')
        if llm_text and llm_text != self.last_printed_text:
            logger.info(f'LLM回复: {llm_text}')
            self.last_printed_text = llm_text
            
            # 解析并执行指令
            self.parse_and_execute_command(llm_text)
        else:
            logger.debug('LLM文本与上次相同，跳过更新')
        logger.info('LLM消息处理完成')
        
    def _init_publishers(self):
        # Home control command publisher
        self.home_control_pub = self.create_publisher(
            String, 'home_control_command', 10)
        
        # Voice command publisher (for life assistance, etc.)
        if VOICE_MSGS_AVAILABLE:
            self.voice_command_pub = self.create_publisher(
                VoiceCommand, 'voice_command', 10)
        else:
            self.voice_command_pub = None

        # [NEW] Motion Control Publisher
        self.motion_pub = self.create_publisher(
            String, 'motion_command', 10)

        # [NEW] System Control Publisher
        self.system_pub = self.create_publisher(
            String, 'system_command', 10)

        # [NEW] Media Control Publisher
        self.media_pub = self.create_publisher(
            String, 'media_command', 10)
            
        logger.info("ROS发布者初始化完成")

    def parse_and_execute_command(self, text):
        """解析并执行指令"""
        if not text:
            return

        # 1. 尝试解析 [INTENT:ID:SLOTS] 格式 (新标准)
        # Regex to match [INTENT:ID:SLOTS]
        # ID might contain underscores, e.g. SAFETY_STOP
        intent_matches = re.findall(r'\[INTENT:([A-Za-z0-9_]+):?(.*?)\]', text)
        if intent_matches:
            for intent_id, slots in intent_matches:
                self._execute_intent(intent_id, slots)
            return

        # 2. 尝试解析 [CMD:domain:action:params] 格式 (旧标准, 兼容)
        matches = re.findall(r'\[CMD:(.*?)\]', text)
        for match in matches:
            try:
                logger.info(f"解析到指令: {match}")
                parts = match.split(':')
                if len(parts) >= 2:
                    domain = parts[0]
                    action = parts[1]
                    params = ':'.join(parts[2:]) if len(parts) > 2 else ""
                    
                    if domain == "home":
                        self._execute_home_command(action, params)
                    elif domain == "life":
                        self._execute_life_command(action, params)
                    else:
                        logger.warning(f"未知指令域: {domain}")
            except Exception as e:
                logger.error(f"指令执行失败: {match}, 错误: {e}")

    def _execute_intent(self, intent_id, slots):
        """执行意图指令"""
        logger.info(f"执行意图: {intent_id}, 槽位: {slots}")
        
        # 紧急安全
        if intent_id.startswith("SAFETY_"):
            if intent_id == "SAFETY_SOS":
                # 同时触发本地报警和Life Assistance的SOS流程
                self._execute_life_command("sos", "")
            else:
                self.motion_pub.publish(String(data=intent_id))
        
        # 运动控制
        elif intent_id.startswith("MOVE_"):
            self.motion_pub.publish(String(data=intent_id))
            
        # 音量/媒体
        elif intent_id.startswith("VOL_") or intent_id.startswith("MEDIA_"):
            # 将槽位与ID组合发送，如 VOL_UP:20
            command_data = f"{intent_id}:{slots}" if slots else intent_id
            self.media_pub.publish(String(data=command_data))
            
        # 工具助手 (Time/Date 直接本地回复)
        elif intent_id == "TIME_QUERY":
            now_time = datetime.now().strftime("%p %I点%M分").replace("AM", "上午").replace("PM", "下午")
            self.mqtt_service.publish_tts(f"现在是{now_time}")
            
        elif intent_id == "DATE_QUERY":
            weekdays = ["星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日"]
            now = datetime.now()
            date_str = now.strftime("%m月%d日")
            weekday = weekdays[now.weekday()]
            self.mqtt_service.publish_tts(f"今天是{date_str}，{weekday}")
            
        elif intent_id == "ALARM_STOP":
            # 发送给 Life Assistance
            if self.voice_command_pub:
                 msg = VoiceCommand()
                 msg.command = "ALARM_STOP"
                 msg.language = "cmd"
                 msg.confidence = 1.0
                 self.voice_command_pub.publish(msg)

        # 系统控制
        elif intent_id.startswith("SYS_") or intent_id == "NET_CONFIG":
            self.system_pub.publish(String(data=intent_id))
            
        else:
            logger.warning(f"未处理的意图: {intent_id}")

    def _execute_home_command(self, action, params):
        """执行家居控制指令"""
        # 构造 control_device:<device_id>:<command>:<value> 格式
        # 假设 params 格式为 device_id:value 或者只是 device_id (默认 value)
        
        device_id = ""
        value = "default"
        
        # 简单解析 params
        param_parts = params.split(':')
        if len(param_parts) >= 1:
            device_id = param_parts[0]
        if len(param_parts) >= 2:
            value = param_parts[1]
            
        command_str = f"control_device:{device_id}:{action}:{value}"
        
        msg = String()
        msg.data = command_str
        self.home_control_pub.publish(msg)
        logger.info(f"已发布家居控制指令: {command_str}")

    def _execute_life_command(self, action, params):
        """执行生活辅助指令"""
        if not self.voice_command_pub:
            logger.warning("VoiceCommand publisher unavailable")
            return

        final_command = params
        
        # 1. 设置用药提醒
        if action == "set_medication":
            # format: set_medication:0800:降压药
            # params 可能是 0800:降压药
            final_command = f"set_medication:{params}"
            
        # 2. 触发SOS
        elif action == "sos":
            final_command = "触发SOS"
            
        # 3. 设置闹钟 (兼容旧逻辑)
        elif action == "set_alarm":
             if "tomorrow" in params:
                 time_part = params.replace("tomorrow_", "")
                 if len(time_part) == 4:
                     hour = time_part[0:2]
                     minute = time_part[2:4]
                     final_command = f"明天{hour}点{minute}分设置闹钟"
             elif "today" in params:
                 time_part = params.replace("today_", "")
                 if len(time_part) == 4:
                     hour = time_part[0:2]
                     minute = time_part[2:4]
                     final_command = f"{hour}点{minute}分设置闹钟"
        
        msg = VoiceCommand()
        msg.command = final_command
        msg.language = "zh-CN"
        msg.confidence = 1.0  # LLM生成的指令置信度设为最高
        
        self.voice_command_pub.publish(msg)
        logger.info(f"已发布语音指令: {final_command}")
            
    def handle_goodbye_message(self, message):
        """处理GOODBYE消息，用于结束会话
        
        Args:
            message (dict): GOODBYE消息内容
        """
        logger.info(f'处理GOODBYE消息: {message}')
        
        with self.lock:
            message_session_id = message.get('session_id')
            logger.debug(f'GOODBYE消息会话ID: {message_session_id}, 当前会话ID: {self.session_id}')
            
            # 检查会话ID是否匹配
            if message_session_id == self.session_id:
                logger.info(f'结束会话，会话ID: {self.session_id}')
                self.session_id = None  # 重置会话ID
                logger.debug('正在停止音频服务...')
                self.audio_service.stop()  # 停止音频服务
                logger.info('音频服务已停止')
            else:
                logger.warning(f'会话ID不匹配，忽略GOODBYE消息')
        
        logger.info('GOODBYE消息处理完成')
    
    def _on_listen_start(self):
        """监听开始回调"""
        logger.info("开始监听语音")
        self.get_logger().debug("开始监听")
        
        # 检查会话ID
        if not self.session_id:
            # 会话ID为空，建立会话
            print("连接会话…")
            logger.info("会话ID为空，发起会话请求")
            
            # 发送HELLO消息建立会话
            self.send_hello_message()
            
        # 发送LISTEN start消息，通知服务器开始录音
        self.send_listen_message("start")
    
    def _on_listen_stop(self):
        """监听停止回调"""
        logger.info("停止监听语音")
        self.get_logger().debug("停止监听")
        
        # 发送LISTEN stop消息，通知服务器停止录音
        self.send_listen_message("stop")
        
        # 记录上次监听停止时间，用于会话超时检查
        self.last_listen_stop_time = time.time()
        logger.debug(f'更新上次监听停止时间: {self.last_listen_stop_time}')
    
    def _on_wake_word_detected(self):
        """唤醒词检测回调"""
        logger.info("🎉 检测到唤醒词'小智'，开始正常音频处理")
        print("👋 你好！我是小智，请问有什么可以帮助你的？")


def main(args=None):
    """主函数，启动小智节点
    
    Args:
        args: 命令行参数
    """
    logger.info('正在启动小智节点...')
    # 初始化ROS 2
    rclpy.init(args=args) 
    # 创建小智节点实例
    logger.debug('创建小智节点实例...')
    node = XiaoZhiNode()
    logger.info('小智节点实例创建完成')
    
    try:
        rclpy.spin(node)  # 运行节点，处理事件
    except KeyboardInterrupt:
        # 处理键盘中断
        logger.info('收到键盘中断，停止节点...')
        node.get_logger().info('Interrupted')
    except Exception as e:
        logger.exception(f'节点运行过程中发生错误: {e}')
    finally:
        # 清理资源
        logger.info('开始清理资源...')
        
        logger.debug('停止音频服务...')
        node.audio_service.stop()
        logger.info('音频服务已停止')
        
        logger.debug('停止MQTT服务...')
        node.mqtt_service.stop()
        logger.info('MQTT服务已停止')
        
        logger.debug('销毁节点...')
        node.destroy_node()
        logger.info('节点已销毁')        
        rclpy.shutdown()
        logger.info('小智节点已停止运行')

