#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
唤醒词检测器 - Sherpa-onnx 高性能版
使用 sherpa-onnx 关键词检测（比 Vosk 更快、更轻量）
采用多线程架构，避免 I/O 阻塞影响检测延迟
"""

import logging
import logging
import time
import numpy as np
import sherpa_onnx

logger = logging.getLogger(__name__)


class IntegratedWakeWordDetector:
    """唤醒词检测器，使用 Sherpa-onnx 关键词检测
    
    外部调用 feed_audio() 输入音频数据
    
    高性能特性:
    - 使用 sherpa-onnx 关键词检测引擎
    """
    
    def __init__(
        self, 
        model_dir,
        keywords_file,
        sample_rate
    ):
        """初始化检测器
        
        Args:
            model_dir (str): Sherpa-onnx 模型目录
            keywords_file (str): 关键词文件路径
        """
        self.sample_rate = sample_rate
        
        # 消除硬编码
        self.INT16_MAX = 32768.0
        
        # 初始化 Sherpa-onnx 关键词检测器
        logger.info(f"加载 Sherpa-onnx 模型: {model_dir}")
        
        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=f"{model_dir}/tokens.txt",
            encoder=f"{model_dir}/encoder-epoch-12-avg-2-chunk-16-left-64.onnx",
            decoder=f"{model_dir}/decoder-epoch-12-avg-2-chunk-16-left-64.onnx",
            joiner=f"{model_dir}/joiner-epoch-12-avg-2-chunk-16-left-64.onnx",
            num_threads=4,
            max_active_paths=2,
            keywords_file=keywords_file,
            keywords_score=1.5,
            keywords_threshold=0.35,
            num_trailing_blanks=1,
            provider="cpu",
        )
        
        # 创建流
        self._stream = self._spotter.create_stream()
        
        # 冷却时间参数
        self.last_detected_time = 0
        self.detection_cooldown = 1.0  # 检测后1秒内不重复触发
        
        logger.info(f"唤醒词检测器初始化完成")
    
    def reset(self):
        """重置检测器状态"""
        logger.info("重置唤醒词检测器状态")
        self.last_detected_time = 0
        # 重置 stream
        self._spotter.reset_stream(self._stream)
        logger.info("唤醒词检测器已重置")
    
    def feed_audio(self, frame_bytes):
        """处理一帧音频数据（集成模式使用）
        
        Args:
            frame_bytes (bytes): PCM16LE格式的音频帧数据
            
        Returns:
            str or None: 检测到的唤醒词文本，未检测到返回 None
        """
        # 检查冷却时间
        current_time = time.time()
        if current_time - self.last_detected_time < self.detection_cooldown:
            return None
        
        # 转换为 float32 范围 [-1, 1]
        samples = np.frombuffer(frame_bytes, dtype=np.int16).astype(np.float32) / self.INT16_MAX
        
        # 输入到检测器
        self._stream.accept_waveform(self.sample_rate, samples)
        
        while self._spotter.is_ready(self._stream):
            self._spotter.decode_stream(self._stream)
        
        # 检查是否触发唤醒词
        result = self._spotter.get_result(self._stream)
        if result:
            logger.info(f"检测到唤醒词: {result}")
            self.last_detected_time = current_time
            # 重置 stream
            self._spotter.reset_stream(self._stream)
            return result.strip()
        
        return None