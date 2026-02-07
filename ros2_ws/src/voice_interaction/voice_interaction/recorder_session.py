import time
from collections import deque

import numpy as np

from llm_node.vad_detector import VadEvent


class RecorderSession:
    """录音会话，封装 VAD 分帧、预滚缓存和结束条件。"""

    def __init__(
        self,
        vad,
        sample_rate,
        vad_frame_ms,
        max_buffer_frames,
        no_speech_timeout,
        pre_roll_frames=15,
    ):
        self.vad = vad
        self.sample_rate = sample_rate
        self.vad_frame_ms = vad_frame_ms
        self.max_buffer_frames = max_buffer_frames
        self.no_speech_timeout = no_speech_timeout
        self.pre_roll_frames = pre_roll_frames

        self.audio_buffer = []
        self.pre_roll_buffer = deque(maxlen=pre_roll_frames)
        self.recording_started = False
        self.recording_start_time = 0.0

    def reset_for_new_recording(self):
        self.audio_buffer.clear()
        self.pre_roll_buffer.clear()
        self.vad.reset()
        self.recording_started = False
        self.recording_start_time = time.time()

    def clear(self):
        self.audio_buffer.clear()
        self.pre_roll_buffer.clear()
        self.recording_started = False

    def pop_audio_buffer(self):
        buffer_data = list(self.audio_buffer)
        self.audio_buffer.clear()
        return buffer_data

    def frame_count(self):
        return len(self.audio_buffer)

    def process_frame(self, data):
        """处理单帧 PCM，返回事件与新增可发送帧。
        
        该方法实现了 VAD 检测、Pre-roll 回溯补全以及录音数据缓存。
        同时支持"批量上传"(通过 audio_buffer) 和 "流式上传"(通过 new_frames)。
        
        Args:
            data (bytes): 60ms 的 PCM 音频帧 (960 samples @ 16kHz)
            
        Returns:
            dict: 包含以下字段:
                - listen_started (bool): 是否刚刚检测到语音开始
                - end_detected (bool): 是否检测到语音结束
                - new_frames (list[bytes]): 本次产生的待发送帧列表 (包含 Pre-roll 和当前帧)
                - no_speech_timeout (bool): 是否等待语音超时
                - max_duration (bool): 是否达到最大录音时长
        """
        # 1. 将 60ms 大帧切分为 VAD 所需的小帧 (20ms/30ms)
        # 此处根据 VAD 配置计算小帧样本数
        samples = np.frombuffer(data, dtype=np.int16)
        vad_frame_samples = int(self.sample_rate * self.vad_frame_ms / 1000)

        start_detected = False
        end_detected = False

        # 逐个切片喂给 VAD 引擎
        for i in range(0, len(samples) - vad_frame_samples + 1, vad_frame_samples):
            chunk = samples[i:i + vad_frame_samples].tobytes()
            chunk_event = self.vad.feed(chunk)
            if chunk_event == VadEvent.START:
                start_detected = True
            elif chunk_event == VadEvent.END:
                end_detected = True

        listen_started = False
        new_frames = []

        # 2. 处理语音开始 (VAD START)
        # 如果 VAD 检测到开始，且之前未开始录音 -> 触发启动逻辑
        if start_detected and not self.recording_started:
            self.recording_started = True
            listen_started = True

            # 回溯机制 (Pre-roll):
            # 将 VAD 触发前的一小段历史音频 (Pre-roll Buffer) 取出
            # 作用: 防止语音开头的第一个字被截断
            pre_roll_frames = list(self.pre_roll_buffer)
            for pre_frame in pre_roll_frames:
                self.audio_buffer.append(pre_frame)  # 存入全量缓存 (Batch用)
                new_frames.append(pre_frame)         # 加入增量列表 (Stream用)
            self.pre_roll_buffer.clear()

        # 3. 数据存入
        if not self.recording_started:
            # 未开始录音: 存入环形缓冲区 (自动淘汰旧帧)，为 Pre-roll 做准备
            self.pre_roll_buffer.append(data)
        else:
            # 录音中: 存入全量缓存和增量列表
            self.audio_buffer.append(data)
            new_frames.append(data)

        # 4. 构建返回结果
        result = {
            "listen_started": listen_started,
            "end_detected": end_detected and self.recording_started, # 只有在录音中检测到 END 才算有效
            "no_speech_timeout": False,
            "max_duration": False,
            "new_frames": new_frames, # 外部拿着这个列表去遍历发送 (流式)
        }

        # 5. 超时检查
        if not self.recording_started:
            # 还没开始说话: 检查"无语音超时" (如用户一直不说话)
            if time.time() - self.recording_start_time > self.no_speech_timeout:
                self.clear()
                result["no_speech_timeout"] = True
        elif len(self.audio_buffer) >= self.max_buffer_frames:
            # 正在说话: 检查"最大录音时长" (防止无限录音)
            result["max_duration"] = True

        return result
