from dataclasses import dataclass


@dataclass
class Config:
    """配置类，定义了系统的各项参数设置"""
    # MQTT相关配置
    ota_url: str = 'https://api.tenclass.net/xiaozhi/ota/'  # OTA更新服务器地址，用于获取MQTT连接信息
    reconnect_interval: int = 5                           # MQTT重连间隔时间（秒）
    heartbeat_interval: int = 30                          # 心跳发送间隔时间（秒）
    # 音频相关配置
    mic_device: str = "plughw:0,1"                        # 麦克风设备
    spk_device: str = "plughw:0,0"                        # 扬声器设备
    sample_rate: int = 16000                             # 音频采样率（Hz） - 改为原生 16kHz
    vad_frame_ms: int = 20                               # VAD检测帧长度（毫秒）
    vad_mode: int = 3                                    # VAD检测灵敏度（0-3，数值越大越敏感）- 提高到最高灵敏度
    vad_start_frames: int = 5                            # 触发VAD开始的连续语音帧数量 - 减少到5帧（100ms）
    vad_end_silence_ms: int = 1000                       # VAD结束的静音时长（毫秒）
    vad_pre_roll_ms: int = 500                           # VAD触发前预存的音频时长（毫秒）- 增加到500ms确保不漏掉开头
    frame_duration: int = 60                             # 帧持续时间（毫秒）

    # 唤醒词检测相关配置 (Sherpa-onnx)
    sherpa_model_dir: str = "/root/sherpa-models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"  # Sherpa-onnx模型目录
    sherpa_keywords_file: str = "/root/sherpa-models/keywords.txt"  # 关键词文件路径
    wake_word: str = "小光"                               # 唤醒词（用于日志显示）
    
    # 视觉相关配置
    camera_index: int = 0                                 # 摄像头设备索引
    face_detection_method: str = "haar"                   # 人脸检测方法：'haar' 或 'dnn'
    face_detection_interval: float = 1.0                  # 人脸检测间隔（秒）
    greeting_cooldown: float = 30.0                       # 主动问候冷却时间（秒）
    vision_enabled: bool = True                           # 是否启用视觉服务
    
    @property
    def aes_opus_info(self):
        """生成AES-OPUS音频流配置信息，用于会话建立时的握手消息"""
        return {
            "type": "hello",                           # 消息类型：会话建立
            "version": 3,                               # 协议版本
            "transport": "udp",                        # 传输方式：UDP
            "udp": {
                "server": '120.24.160.13',              # UDP服务器地址
                "port": 8884,                  # UDP服务器端口
                "encryption": "aes-128-ctr",           # 加密方式：AES-128-CTR
                "key": '263094c3aa28cb42f3965a1020cb21a7',                        # 加密密钥
                "nonce": '01000000ccba9720b4bc268100000000'                     # 加密随机数
            },
            "audio_params": {
                "format": "opus",                      # 音频格式：Opus
                "sample_rate": self.sample_rate,        # 采样率
                "channels": 1,                          # 声道数：单声道
                "frame_duration": 60                    # 帧持续时间（毫秒）
            },
            "session_id": None                          # 会话ID，初始化为None
        }
    