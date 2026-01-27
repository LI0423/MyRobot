#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的唤醒词检测测试脚本
专注于检测唤醒词"小智"
"""

import sys
import os
import logging
import pyaudio
import vosk
import json
import time

# 配置日志
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class SimpleWakeWordDetector:
    """简单的唤醒词检测器"""

    def __init__(self, model_path, wake_word="小光"):
        """初始化检测器"""
        self.wake_word = wake_word
        self.chunk_size = 8000  # 400ms的音频

        # 初始化PyAudio
        self.pa = pyaudio.PyAudio()

        # 查找麦克风设备并检测支持的采样率
        self.mic_device = None
        self.sample_rate = None

        for i in range(self.pa.get_device_count()):
            dev_info = self.pa.get_device_info_by_index(i)
            if dev_info["maxInputChannels"] > 0:
                self.mic_device = i

                # 尝试获取设备支持的采样率
                # 先尝试常用的采样率
                supported_rates = [16000, 44100, 48000, 8000, 22050]
                for rate in supported_rates:
                    try:
                        # 尝试打开流来检测采样率是否支持
                        stream = self.pa.open(
                            rate=rate,
                            channels=1,
                            format=pyaudio.paInt16,
                            input=True,
                            input_device_index=self.mic_device,
                            frames_per_buffer=1024,
                        )
                        stream.close()
                        self.sample_rate = rate
                        logger.info(f"麦克风支持采样率: {rate}Hz")
                        break
                    except:
                        continue

                if self.sample_rate is None:
                    # 如果都不支持，使用设备默认采样率
                    self.sample_rate = int(dev_info["defaultSampleRate"])
                    logger.warning(
                        f"无法检测支持的采样率，使用默认采样率: {self.sample_rate}Hz"
                    )
                break

        if self.mic_device is None:
            logger.error("未找到麦克风设备")
            sys.exit(1)

        # 初始化Vosk模型和识别器
        logger.info(f"加载Vosk模型: {model_path}")
        self.model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)

        logger.info(f"使用麦克风: #{self.mic_device}")
        logger.info(f"使用采样率: {self.sample_rate}Hz")
        logger.info(f"✅ 唤醒词检测器初始化完成，唤醒词: '{self.wake_word}'")

    def start_listening(self):
        """开始监听麦克风"""
        logger.info("🎤 开始监听麦克风，说'小光'来测试唤醒...")
        logger.info("按 Ctrl+C 停止")

        # 打开音频流
        stream = self.pa.open(
            rate=self.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            input_device_index=self.mic_device,
            frames_per_buffer=self.chunk_size,
        )

        try:
            while True:
                # 读取音频数据
                data = stream.read(self.chunk_size, exception_on_overflow=False)

                # 发送音频数据给识别器
                if self.recognizer.AcceptWaveform(data):
                    # 获取识别结果
                    result = json.loads(self.recognizer.Result())
                    print(result)
                    text = result.get("text", "").strip()

                    if text:
                        logger.info(f"识别到: '{text}'")

                        # 检查唤醒词
                        if self.wake_word in text:
                            logger.info("✅ 唤醒成功!")
                            # break
                else:
                    # 获取部分结果
                    partial = json.loads(self.recognizer.PartialResult())
                    partial_text = partial.get("partial", "").strip()
                    if partial_text:
                        # 在同一行显示部分结果
                        sys.stdout.write(f"\r正在听: {partial_text}")
                        sys.stdout.flush()

                        # 检查唤醒词
                        if self.wake_word in partial_text:
                            logger.info("✅ 唤醒成功!")
                            # break

        except KeyboardInterrupt:
            logger.info("\n⏹️  用户中断")
        finally:
            # 关闭音频流
            stream.stop_stream()
            stream.close()
            self.pa.terminate()
            logger.info("✅ 音频流已关闭")


def main():
    """主函数"""
    logger.info("Vosk 唤醒词检测简单测试")

    # 设置模型路径
    model_path = "/root/vosk-model-small-cn-0.22"

    # 创建检测器
    detector = SimpleWakeWordDetector(model_path, wake_word="小光")

    # 开始监听
    detector.start_listening()


if __name__ == "__main__":
    main()
