import json
import ssl
import threading
import time
import logging
import paho.mqtt.client as mqtt
from llm_node.utils import get_ota_version
from llm_node.config import Config

logger = logging.getLogger(__name__)

# 常量定义
HEARTBEAT_INTERVAL = 30  # 心跳发送间隔时间（秒）


class MQTTService:
    """MQTT服务类，负责处理与MQTT服务器的连接、消息订阅和发布"""

    def __init__(self, config: Config):
        """初始化MQTT服务
        
        Args:
            config (Config): 配置对象，包含MQTT相关配置
        """
        logger.info('正在初始化MQTT服务...')
        self.config = config
        
        self._on_message = None                  # 消息回调函数
        self._thread = None                      # 主线程引用
        self._running = False                    # 服务运行状态
        self._heartbeat_thread = None            # 心跳线程引用
        self._heartbeat_running = False          # 心跳线程运行状态
        self.last_heartbeat = None               # 上次发送心跳的时间
        self.last_listen_stop_time = None        # 上次监听停止的时间
        self.client = None                       # MQTT客户端对象
        self.publish_topic = None                # 发布主题
        self.subscribe_topic = None              # 订阅主题
        
        try:
            # 从OTA服务器获取MQTT连接信息
            logger.debug(f'从OTA服务器获取MQTT连接信息，URL: {config.ota_url}')
            self.mqtt_info = get_ota_version(config.ota_url)
            logger.info(f'成功获取MQTT连接信息: {self.mqtt_info}')
            
            self.publish_topic = self.mqtt_info['publish_topic']
            self.subscribe_topic = self.mqtt_info['subscribe_topic']
            logger.info(f'MQTT主题: 发布={self.publish_topic}, 订阅={self.subscribe_topic}')
        except Exception as e:
            logger.error(f"Failed to get MQTT info: {e}")
            # 初始化默认值
            self.mqtt_info = {'publish_topic': '', 'subscribe_topic': ''}
            self.publish_topic = ''
            self.subscribe_topic = ''
            logger.warning('使用默认MQTT主题')
        logger.info('MQTT服务初始化完成')

    def set_message_callback(self, callback):
        """设置消息回调函数
        
        Args:
            callback (callable): 处理接收到的MQTT消息的回调函数
        """
        self._on_message = callback

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT连接成功回调函数
        
        Args:
            client: MQTT客户端对象
            userdata: 用户数据
            flags: 连接标志
            rc: 连接结果码
        """
        if rc == 0:
            # 连接成功，订阅主题
            result = self.subscribe()
            logger.info(f"MQTT连接成功，订阅结果: {result}")
        else:
            # 连接失败
            logger.error(f"MQTT连接失败，错误码: {rc}")
            
    def _on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调函数
        
        Args:
            client: MQTT客户端对象
            userdata: 用户数据
            rc: 断开原因码
        """
        if not self._running:
            return
        logger.warning(f"MQTT连接断开，错误码: {rc}，正在尝试重连...")
        try:
            # 尝试重新连接
            self.client.reconnect()
        except Exception as e:
            logger.error(f"重连失败: {str(e)}")

    def _on_message_inner(self, client, userdata, msg):
        """内部消息处理函数，用于解析和分发MQTT消息
        
        Args:
            client: MQTT客户端对象
            userdata: 用户数据
            msg: MQTT消息对象
        """
        try:
            # 解析消息负载
            payload = msg.payload.decode('utf-8') if isinstance(msg.payload, bytes) else str(msg.payload)
            data = json.loads(payload)
        except Exception:
            # 解析失败时设置为None
            data = None
        if self._on_message:
            try:
                # 调用外部设置的消息回调函数
                self._on_message(msg.topic, data, msg)
            except Exception as e:
                logger.exception("Error in user on_message")
    
    def _send_heartbeat(self):
        """心跳和会话超时检查线程
        
        定期发送心跳消息，并检查会话超时
        """
        # 初始化心跳时间
        self.last_heartbeat = time.time()
        
        while self._heartbeat_running:
            # 发送心跳
            if (self.client and self.client.is_connected() and self.publish_topic and 
                time.time() - self.last_heartbeat > HEARTBEAT_INTERVAL):
                try:
                    # 发布心跳消息
                    self.client.publish(self.publish_topic, json.dumps({"type": "heartbeat"}))
                    self.last_heartbeat = time.time()
                    logger.debug("心跳已发送")
                except Exception as e:
                    logger.error(f"心跳发送失败: {str(e)}")

            # 检查会话超时
            if (self.last_listen_stop_time is not None and 
                time.time() - self.last_listen_stop_time > 5):
                logger.info("会话超时，关闭会话")
                # 重置超时时间
                self.last_listen_stop_time = None

            time.sleep(1)

    def start(self):
        """启动MQTT服务
        
        初始化MQTT客户端，建立连接，并启动心跳线程
        """
        logger.info('正在启动MQTT服务...')
        self._running = True
        
        # 创建MQTT客户端对象
        logger.debug('创建MQTT客户端对象...')
        client_id = self.mqtt_info.get('client_id', 'default_client')
        logger.debug(f'使用客户端ID: {client_id}')
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, 
                                  client_id=client_id)
        logger.info('MQTT客户端对象创建完成')
        
        # 设置用户名和密码
        username = self.mqtt_info.get('username', 'default_user')
        logger.debug(f'设置用户名和密码，用户: {username}')
        self.client.username_pw_set(username=username, password=self.mqtt_info.get('password', ''))
        logger.info('用户名和密码设置完成')
        
        # 配置TLS
        logger.debug('配置TLS...')
        ctx = ssl.create_default_context()
        ctx.check_hostname = False  # 不检查主机名
        ctx.verify_mode = ssl.CERT_NONE  # 不验证证书
        self.client.tls_set_context(ctx)
        logger.info('TLS配置完成')
        
        # 绑定回调函数
        logger.debug('绑定MQTT回调函数...')
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message_inner
        logger.info('MQTT回调函数绑定完成')

        try:
            # 连接到MQTT服务器
            endpoint = self.mqtt_info.get('endpoint', 'localhost')
            port = self.mqtt_info.get('port', 8883)
            logger.info(f'正在连接到MQTT服务器: {endpoint}:{port}')
            self.client.connect(endpoint, port, 60)
            logger.info(f'成功连接到MQTT服务器: {endpoint}:{port}')
            
            # 启动MQTT客户端循环
            logger.debug('启动MQTT客户端循环...')
            self.client.loop_start()
            logger.info('MQTT客户端循环启动完成')
            
            # 启动心跳线程
            logger.debug('启动心跳线程...')
            self._heartbeat_running = True
            self._heartbeat_thread = threading.Thread(target=self._send_heartbeat, daemon=True, name="mqtt_heartbeat_thread")
            self._heartbeat_thread.start()
            logger.info('心跳线程启动完成')
            
            logger.info("MQTT client started")
        except Exception as e:
            logger.exception(f"MQTT start failed: {e}")
            self._running = False

    def stop(self):        
        """停止MQTT服务
        
        停止心跳线程，断开MQTT连接
        """
        logger.info('正在停止MQTT服务...')
        self._running = False
        
        # 停止心跳线程
        logger.debug('停止心跳线程...')
        self._heartbeat_running = False
        if self._heartbeat_thread:
            logger.debug('等待心跳线程结束...')
            self._heartbeat_thread.join(timeout=2.0)
            logger.info('心跳线程已结束')
        
        if self.client:
            try:
                logger.debug('停止MQTT客户端循环...')
                # 停止MQTT客户端循环
                self.client.loop_stop()
                logger.info('MQTT客户端循环已停止')
                
                logger.debug('断开MQTT连接...')
                # 断开连接
                self.client.disconnect()
                logger.info('MQTT连接已断开')
            except Exception as e:
                logger.exception("Error stopping MQTT client")
        
        logger.info("MQTT client stopped")

    def subscribe(self):
        """订阅主题
        
        Returns:
            tuple: 订阅结果和消息ID，如果订阅失败则返回None
        """
        logger.info(f'正在订阅主题: {self.subscribe_topic}')
        if self.client and self.client.is_connected():
            logger.debug(f'客户端已连接，开始订阅主题: {self.subscribe_topic}')
            result, mid = self.client.subscribe(self.subscribe_topic)
            logger.info(f'主题订阅完成，结果: {result}, 消息ID: {mid}')
            return result, mid
        else:
            logger.warning(f'无法订阅主题，客户端未连接: {self.subscribe_topic}')
        return None

    def publish(self, payload, topic=None):
        """发布消息到指定主题，如果未指定主题则使用publish_topic
        
        Args:
            payload: 消息内容
            topic (str, optional): 目标主题，默认为None，使用publish_topic
            
        Returns:
            tuple: 发布结果和消息ID，如果发布失败则返回None
        """
        # 如果未指定主题，使用publish_topic
        publish_topic = topic if topic is not None else self.publish_topic
        logger.info(f'正在发布消息到主题: {publish_topic}, 内容: {payload}')
        
        if self.client and self.client.is_connected():
            try:
                # 如果是字典或列表，转换为JSON字符串
                if isinstance(payload, (dict, list)):
                    logger.debug('将有效载荷转换为JSON字符串...')
                    payload = json.dumps(payload)
                    logger.debug(f'转换后的JSON: {payload}')
                
                logger.debug(f'发布消息到主题: {publish_topic}')
                result, mid = self.client.publish(publish_topic, payload)
                logger.info(f'消息发布完成，结果: {result}, 消息ID: {mid}')
                return result, mid
            except Exception as e:
                logger.exception("MQTT publish failed")
        else:
            logger.warning(f'无法发布消息，客户端未连接: {publish_topic}')
        return None



