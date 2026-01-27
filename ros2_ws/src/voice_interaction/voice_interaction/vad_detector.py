import logging
import webrtcvad  # 导入WebRTC VAD库，用于语音活动检测
import collections  # 导入collections模块，用于实现预滚缓冲区


class VadEvent:
    """VAD事件类型枚举，定义了语音活动检测的三种状态"""
    START = 'start'  # 检测到语音开始
    END = 'end'      # 检测到语音结束
    NONE = 'none'    # 无语音状态变化
    
class VADDetector:
    """语音活动检测器，基于WebRTC VAD实现"""
    def __init__(self, sample_rate=16000, frame_ms=20, mode=2, start_frames=10, end_silence_ms=1000, pre_roll_ms=300):
        """初始化VAD检测器
        
        Args:
            sample_rate (int): 音频采样率（Hz）
            frame_ms (int): 每帧音频长度（毫秒）
            mode (int): VAD检测灵敏度（0-3，数值越大越敏感）
            start_frames (int): 触发语音开始的连续语音帧数量
            end_silence_ms (int): 触发语音结束的静音时长（毫秒）
            pre_roll_ms (int): 语音开始前预存的音频时长（毫秒）
        """
        self.sample_rate = sample_rate  # 音频采样率
        self.frame_ms = frame_ms        # 每帧音频长度
        self.vad = webrtcvad.Vad(mode)  # 初始化WebRTC VAD对象
        self.start_frames = start_frames  # 触发语音开始的连续语音帧数量
        self.end_silence_ms = end_silence_ms  # 触发语音结束的静音时长
        self.pre_roll_frames = max(1, int(pre_roll_ms / frame_ms))  # 预滚缓冲区的帧数
        
        self.state = 'idle'  # 当前状态：'idle'（空闲）或'speaking'（说话中）
        self.speech_frame_count = 0  # 连续语音帧计数
        self.silence_ms = 0  # 连续静音时长（毫秒）
        self.pre_roll = collections.deque(maxlen=self.pre_roll_frames)  # 预滚缓冲区，存储最近的音频帧
        
    def reset(self):
        """重置VAD检测器状态"""
        self.state = 'idle'  # 重置为空闲状态
        self.speech_frame_count = 0  # 重置连续语音帧计数
        self.silence_ms = 0  # 重置连续静音时长
        self.pre_roll.clear()  # 清空预滚缓冲区
        
    def feed(self, frame_bytes):
        """输入一帧PCM16LE格式的音频数据，进行VAD检测
        
        Args:
            frame_bytes (bytes): PCM16LE格式的音频帧数据
            
        Returns:
            VadEvent: 检测结果，可能是START、END或NONE
        """
        is_speech = False  # 语音检测结果
        try:
            # WebRTC VAD只支持特定的帧大小：10ms, 20ms, 30ms
            # 根据采样率计算每帧的样本数，确保是支持的大小
            frame_size = len(frame_bytes) // 2  # 每样本2字节
            frame_ms = int(frame_size * 1000 / self.sample_rate)
            
            # 确保帧大小是WebRTC VAD支持的
            if frame_ms not in [10, 20, 30]:
                logging.warning(f"不支持的VAD帧大小: {frame_ms}ms, 采样率: {self.sample_rate}, 样本数: {frame_size}")
                return VadEvent.NONE
            
            # 使用WebRTC VAD检测当前帧是否为语音
            is_speech = self.vad.is_speech(frame_bytes, self.sample_rate)
            logging.debug(f"VAD检测结果: is_speech={is_speech}, 样本数={frame_size}, 采样率={self.sample_rate}, 帧大小={frame_ms}ms")
        except Exception as e:
            # 检测失败时默认为非语音
            logging.error(f"VAD检测失败: {e}")
            is_speech = False

        # 将当前帧添加到预滚缓冲区
        self.pre_roll.append(frame_bytes)

        if self.state == 'idle':
            # 空闲状态下的处理
            if is_speech:
                # 检测到语音，增加连续语音帧计数
                self.speech_frame_count += 1
                if self.speech_frame_count >= self.start_frames:
                    # 连续语音帧达到阈值，切换到说话状态
                    self.state = 'speaking'
                    self.silence_ms = 0
                    # 返回语音开始事件，预滚缓冲区保留供调用者使用
                    return VadEvent.START
            else:
                # 检测到静音，重置连续语音帧计数
                self.speech_frame_count = 0
                return VadEvent.NONE

        elif self.state == 'speaking':
            # 说话状态下的处理
            if is_speech:
                # 检测到语音，重置连续静音时长
                self.silence_ms = 0
            else:
                # 检测到静音，增加连续静音时长
                self.silence_ms += self.frame_ms
                if self.silence_ms >= self.end_silence_ms:
                    # 连续静音达到阈值，切换到空闲状态
                    self.state = 'idle'
                    self.speech_frame_count = 0
                    self.silence_ms = 0
                    # 只有当静音时长达到阈值时，才返回语音结束事件
                    return VadEvent.END
            return VadEvent.NONE

        return VadEvent.NONE

    def get_pre_roll(self):
        """获取预滚缓冲区中的音频帧
        
        Returns:
            list: 预滚缓冲区中的音频帧列表
        """
        return list(self.pre_roll)
