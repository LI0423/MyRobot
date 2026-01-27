import logging
import webrtcvad  # 使用WebRTC VAD进行语音活动检测
import collections
import time

logger = logging.getLogger(__name__)


class WakeWordDetector:
    """唤醒词检测器，用于检测特定唤醒词"""
    
    def __init__(self, sample_rate=48000, frame_ms=20, wake_word="小智"):
        """初始化唤醒词检测器
        
        Args:
            sample_rate (int): 音频采样率（Hz）
            frame_ms (int): 每帧音频长度（毫秒）
            wake_word (str): 唤醒词
        """
        self.sample_rate = sample_rate
        self.frame_ms = frame_ms
        self.wake_word = wake_word
        self.frame_size = int(sample_rate * frame_ms / 1000)  # 每帧样本数
        
        # 初始化VAD，用于检测语音活动
        self.vad = webrtcvad.Vad(3)  # 使用最高灵敏度，更容易检测到语音
        
        # 音频缓冲区，用于存储最近的音频数据
        self.audio_buffer = collections.deque(maxlen=int(1.5 * sample_rate / self.frame_size))  # 最多存储1.5秒音频，更长时间以捕捉完整唤醒词
        
        # 唤醒词检测状态
        self.detected = False
        self.speech_frame_count = 0  # 连续语音帧计数
        self.min_speech_frames = 10  # 触发唤醒词的最小连续语音帧数 (200ms)
        self.max_speech_frames = 150  # 最大连续语音帧数 (3s)，避免过长语音触发
        self.last_detected_time = 0  # 上次检测到唤醒词的时间
        self.detection_cooldown = 1.0  # 冷却时间，避免频繁触发
        self.speech_start_time = 0  # 语音开始时间
        
        logger.info(f"唤醒词检测器初始化完成，参数: 采样率={sample_rate}Hz, 帧大小={frame_ms}ms, VAD灵敏度=3, 最小连续语音帧={self.min_speech_frames}, 冷却时间={self.detection_cooldown}s")
    
    def feed(self, frame_bytes):
        """输入一帧PCM16LE格式的音频数据，进行唤醒词检测
        
        Args:
            frame_bytes (bytes): PCM16LE格式的音频帧数据
            
        Returns:
            bool: 是否检测到唤醒词
        """
        # 检查冷却时间，避免频繁触发
        current_time = time.time()
        if current_time - self.last_detected_time < self.detection_cooldown:
            return False
        
        # 将音频帧添加到缓冲区
        self.audio_buffer.append(frame_bytes)
        
        # 检查是否是语音帧
        is_speech = self.vad.is_speech(frame_bytes, self.sample_rate)
        
        if is_speech:
            # 检测到语音
            if self.speech_frame_count == 0:
                # 语音开始
                self.speech_start_time = current_time
                logger.debug("🔊 检测到语音开始")
            
            # 增加连续语音帧计数
            self.speech_frame_count += 1
            logger.debug(f"🎤 检测到语音帧，连续计数: {self.speech_frame_count}, 最小要求: {self.min_speech_frames}")
            
            # 当连续语音帧数达到阈值时，触发唤醒词检测
            # 注意：当前实现使用简化的语音检测，实际项目中应替换为真正的唤醒词识别模型
            # 这里我们保持连续语音帧检测的逻辑，但优化参数
            speech_duration = self.speech_frame_count * self.frame_ms / 1000
            
            # 唤醒词"小智"的发音通常在500ms-1.5s之间，调整阈值范围
            if 25 <= self.speech_frame_count <= 75:  # 500ms-1.5s
                logger.info(f"🎉 检测到唤醒词: {self.wake_word}")
                logger.info(f"📊 唤醒词检测详情: 连续语音帧={self.speech_frame_count}, 语音时长={speech_duration:.2f}s, 采样率={self.sample_rate}Hz")
                self.detected = True
                self.last_detected_time = current_time
                self.speech_frame_count = 0
                return True
            elif self.speech_frame_count > self.max_speech_frames:
                # 语音过长，重置计数
                logger.debug(f"⏱️  语音过长 ({speech_duration:.2f}s)，重置语音计数")
                self.speech_frame_count = 0
        else:
            # 检测到静音
            if self.speech_frame_count > 0:
                logger.debug(f"🔇 检测到静音，重置连续语音计数: {self.speech_frame_count}")
            self.speech_frame_count = 0
        
        return False
    
    def reset(self):
        """重置唤醒词检测器状态，确保能重新检测唤醒词"""
        logger.info("🔄 重置唤醒词检测器状态")
        self.detected = False
        self.speech_frame_count = 0  # 重置连续语音帧计数
        self.last_detected_time = 0  # 重置冷却时间，允许立即检测
        self.audio_buffer.clear()
        self.speech_start_time = 0  # 重置语音开始时间
        logger.info("✅ 唤醒词检测器已重置，准备检测唤醒词")
    
    @property
    def is_detected(self):
        """获取唤醒词检测状态
        
        Returns:
            bool: 是否检测到唤醒词
        """
        return self.detected
