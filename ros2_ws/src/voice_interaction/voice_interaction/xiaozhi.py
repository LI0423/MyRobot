import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import warnings
import urllib3
import logging
import time

from llm_node.mqtt_service import MQTTService
from llm_node.audio_service import AudioService
from dotenv import load_dotenv
from llm_node.enums import MessageType
from llm_node.config import Config
from common.logging_config import setup_logging
import json

# 屏蔽警告
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)  # 屏蔽urllib3的不安全请求警告
warnings.filterwarnings("ignore", category=DeprecationWarning)  # 屏蔽弃用警告

# 配置统一的日志系统
setup_logging(level=logging.INFO)

# Get logger
logger = logging.getLogger(__name__)

load_dotenv()  # 加载.env文件中的环境变量


class XiaoZhiNode(Node):
    """小智节点主类，整合MQTT服务和音频服务，处理各种消息"""
    
    def __init__(self):
        """初始化小智节点"""
        super().__init__('xiaozhi_node')
        logger.info("正在初始化小智节点...")

        # 初始化配置
        self.config = Config()
        
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
        
        # 创建视觉事件订阅者 (订阅独立的 vision_node 发布的 Topic)
        self.vision_sub = None
        self.vision_mode_pub = None
        if self.config.vision_enabled:
            logger.debug("创建视觉事件订阅者...")
            self.vision_sub = self.create_subscription(
                String,
                '/vision/face_detected',
                self._on_vision_face_detected,
                10
            )
            logger.info("已订阅 /vision/face_detected Topic")
            
            # 创建视觉模式发布者
            self.vision_mode_pub = self.create_publisher(String, '/vision/set_mode', 10)
            logger.info("创建发布者: /vision/set_mode")
        else:
            logger.info("视觉功能已禁用")
        
        # 游戏相关 Topics
        self.game_start_pub = self.create_publisher(String, '/game/rps/start', 10)
        self.game_confirm_pub = self.create_publisher(String, '/game/rps/confirm', 10)
        self.game_state_sub = self.create_subscription(
            String, '/game/rps/state', self._on_game_state, 10)
        self.game_result_sub = self.create_subscription(
            String, '/game/rps/result', self._on_game_result, 10)
        logger.info("游戏 Topics 已初始化")
        
        # 启动MQTT和音频服务的线程
        logger.debug("正在启动服务线程...")
        threading.Thread(target=self._start_mqtt_and_audio, daemon=True, name="service_start_thread").start()
        
        # 状态变量
        self.tts_state = None  # TTS播放状态
        self.last_printed_text = ""  # 上次打印的文本
        self.session_id = None  # 会话ID
        self.lock = threading.Lock()  # 线程锁
        self.last_listen_stop_time = None  # 上次监听停止时间
        self.in_game = False  # 是否在游戏中
        logger.info("小智节点初始化完成")
        
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
        
        logger.debug("正在启动音频服务...")
        self.audio_service.start()
        logger.info("音频服务启动完成")
        
        # 视觉服务现在是独立节点，无需在此启动
        logger.info("注意: 视觉服务是独立节点，请确保 vision_node 已启动")

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
                    "frame_duration": self.config.frame_duration
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
                "session_id": self.session_id,
                "mode": "manual"
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
            print(f"用户: {stt_text}")
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
        else:
            logger.debug('LLM文本与上次相同，跳过更新')
        logger.info('LLM消息处理完成')
            
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
                
                # 重置音频服务回到 IDLE 状态，而不是停止
                # 这样可以继续监听唤醒词
                logger.debug('重置音频服务到 IDLE 状态...')
                self.audio_service.force_idle()
                logger.info('音频服务已重置，继续监听唤醒词')
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
        
        # 切换视觉为高频模式
        self._set_vision_mode('high')
    
    def _on_listen_stop(self):
        """监听停止回调"""
        logger.info("停止监听语音")
        self.get_logger().debug("停止监听")
        
        # 发送LISTEN stop消息，通知服务器停止录音
        self.send_listen_message("stop")
        
        # 记录上次监听停止时间，用于会话超时检查
        self.last_listen_stop_time = time.time()
        logger.debug(f'更新上次监听停止时间: {self.last_listen_stop_time}')
        
        # 切换视觉为低频模式
        self._set_vision_mode('low')
    
    def _on_wake_word_detected(self):
        """唤醒词检测回调
        
        当检测到唤醒词时:
        1. 如果没有会话，发送 HELLO 消息建立会话
        2. 切换视觉为高频模式
        """
        logger.info(f"检测到唤醒词'{self.config.wake_word}'，开始建立会话")
        logger.info(f"你好！我是{self.config.wake_word}，请问有什么可以帮助你的？")
        
        # 检查会话ID，如果没有会话就建立
        if not self.session_id:
            logger.info("连接会话…")
            logger.info("会话ID为空，发送HELLO消息建立会话")
            self.send_hello_message()
        else:
            logger.info(f"会话已存在，会话ID: {self.session_id}")
        
        # 唤醒时切换视觉为高频模式
        self._set_vision_mode('high')
    
    def _set_vision_mode(self, mode):
        """设置视觉检测模式
        
        Args:
            mode (str): 'high' 或 'low'
        """
        if self.vision_mode_pub:
            msg = String()
            msg.data = mode
            self.vision_mode_pub.publish(msg)
            logger.debug(f"发布视觉模式: {mode}")
    
    def _on_vision_face_detected(self, msg):
        """视觉人脸检测消息回调
        
        处理来自 vision_node 的人脸检测事件。
        
        Args:
            msg (String): ROS 2 消息，data 字段为 JSON 格式
        """
        try:
            face_data = json.loads(msg.data)
            face_count = face_data.get('face_count', 0)
            
            if face_count == 0:
                return
            
            logger.info(f"收到视觉事件: 检测到 {face_count} 个人脸")
            
            # 根据检测到的人脸数量生成问候语
            if face_count == 1:
                greeting = f"你好！我是{self.config.wake_word}，很高兴见到你！"
            else:
                greeting = f"你好！我是{self.config.wake_word}，欢迎大家！"
            
            print(greeting)
            
            # 触发语音问候
            if self.audio_service:
                self.audio_service.play_response(greeting)
                
        except json.JSONDecodeError as e:
            logger.error(f"解析视觉消息失败: {e}")
        except Exception as e:
            logger.error(f"处理视觉消息异常: {e}")
    
    # ==================== 游戏相关方法 ====================
    
    def start_rps_game(self):
        """启动猜拳游戏"""
        logger.info("启动猜拳游戏")
        self.in_game = True
        
        msg = String()
        msg.data = 'start'
        self.game_start_pub.publish(msg)
    
    def send_game_confirm(self, confirm_text):
        """发送游戏确认
        
        Args:
            confirm_text (str): 确认文本
        """
        if not self.in_game:
            return
        
        msg = String()
        msg.data = confirm_text
        self.game_confirm_pub.publish(msg)
    
    def _on_game_state(self, msg):
        """处理游戏状态变化
        
        Args:
            msg (String): JSON 格式状态消息
        """
        try:
            data = json.loads(msg.data)
            state = data.get('state')
            message = data.get('message', '')
            
            logger.info(f"游戏状态: {state}, 消息: {message}")
            
            # 播报消息
            if message and self.audio_service:
                print(f"[游戏] {message}")
                self.audio_service.play_response(message)
                
        except Exception as e:
            logger.error(f"处理游戏状态异常: {e}")
    
    def _on_game_result(self, msg):
        """处理游戏结果
        
        Args:
            msg (String): JSON 格式结果消息
        """
        try:
            data = json.loads(msg.data)
            result = data.get('result')
            message = data.get('message', '')
            
            logger.info(f"游戏结果: {result}")
            
            # 播报结果
            if message:
                print(f"[游戏] {message}")
                if self.audio_service:
                    self.audio_service.play_response(message)
            
            # 游戏结束
            self.in_game = False
            
        except Exception as e:
            logger.error(f"处理游戏结果异常: {e}")
    
    def check_game_intent(self, text):
        """检查文本中是否包含游戏意图
        
        Args:
            text (str): 用户输入文本
            
        Returns:
            bool: 是否包含游戏意图
        """
        game_keywords = ['猜拳', '石头剪刀布', '玩游戏', '剪刀石头布']
        return any(keyword in text for keyword in game_keywords)


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

