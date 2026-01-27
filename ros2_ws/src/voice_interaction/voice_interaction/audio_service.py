import threading
import socket
import collections
import pyaudio
import time
import opuslib
import os
import logging
from llm_node.utils import aes_ctr_encrypt, aes_ctr_decrypt
from llm_node.config import Config
from llm_node.vad_detector import VADDetector, VadEvent
from llm_node.wake_word_detector import WakeWordDetector

os.environ['DISPLAY'] = ':0'

logger = logging.getLogger(__name__)

class ALSAErrorSuppressor:
    """ALSA错误输出抑制器，防止音频库错误信息干扰用户界面"""

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


class AudioService:
    """音频服务类，负责处理音频的采集、编码、加密、发送和接收、解密、解码、播放"""

    def __init__(self, config: Config, on_listen_start=None, on_listen_stop=None, on_wake_word_detected=None):
        """初始化音频服务
        
        Args:
            config (Config): 配置对象，包含音频相关配置
            on_listen_start (callable): 监听开始回调函数
            on_listen_stop (callable): 监听停止回调函数
            on_wake_word_detected (callable): 唤醒词检测回调函数
        """
        self.config = config
        self.on_listen_start = on_listen_start  # 监听开始回调
        self.on_listen_stop = on_listen_stop    # 监听停止回调
        self.on_wake_word_detected = on_wake_word_detected  # 唤醒词检测回调

        # 初始化VAD检测器
        self.vad = VADDetector(
            sample_rate=config.sample_rate,
            frame_ms=config.vad_frame_ms,
            mode=config.vad_mode,
            start_frames=config.vad_start_frames,
            end_silence_ms=config.vad_end_silence_ms,
            pre_roll_ms=config.vad_pre_roll_ms
        )
        
        # 初始化唤醒词检测器
        self.wake_word_detector = WakeWordDetector(
            sample_rate=config.sample_rate,
            frame_ms=config.vad_frame_ms,
            wake_word="小智"
        )
        
        # 唤醒状态标志
        self.is_wake_word_detected = False
        
        # 音频播放标志
        self.is_playing = False
        # 音频设备和流
        self.audio = None              # PyAudio对象
        self.spk_stream = None         # 扬声器输出流
        self.mic_stream = None         # 麦克风输入流
        # 线程
        self.send_thread = None        # 音频发送线程
        self.recv_thread = None        # 音频接收线程
        # 状态标志
        self.running = False           # 服务运行状态
        # 网络相关
        self.udp_socket = None         # UDP套接字
        self.lock = threading.Lock()   # 线程锁，保护共享资源
        # 编码器配置
        self.encoder_rate = config.sample_rate                      # 编码器采样率
        self.encoder_frame_ms = config.encoder_frame_ms              # 编码器帧长度（毫秒）
        self.encoder_frame_samples = int(self.encoder_rate * self.encoder_frame_ms / 1000)  # 每帧样本数
        # 音频帧配置
        self.frame_samples = int(self.encoder_rate * config.vad_frame_ms / 1000)  # 每帧样本数（基于VAD帧大小）
        self.frames_per_packet = max(1, self.encoder_frame_samples // self.frame_samples)  # 每个数据包包含的帧数
        # 会话和UDP配置
        self.session_id = None         # 会话ID
        self.udp_info = config.aes_opus_info['udp']
        self.audio_params = config.aes_opus_info['audio_params']
        self.local_sequence = 0        # 本地序列计数器
        
    def update_udp_info(self, udp_dict):
        """更新UDP配置信息
        
        Args:
            udp_dict (dict): 包含UDP配置的字典
        """
        with self.lock:
            self.udp_info.update(udp_dict)
    
    def start(self):
        """启动音频服务"""
        logger.info('正在启动音频服务...')
        if self.running:
            logger.warning('音频服务已在运行，跳过启动')
            return
        
        try:
            # 初始化PyAudio和音频流
            logger.debug('初始化PyAudio...')
            with ALSAErrorSuppressor():
                self.audio = pyaudio.PyAudio()
            logger.info('PyAudio初始化完成')
            
            logger.debug('打开麦克风流...')
            logger.debug(f'音频参数: 采样率={self.config.sample_rate}, 格式=paInt16, 声道=1')
            # 使用卡片1的设备（duplex-audio ES8326 HiFi），并抑制ALSA错误
            with ALSAErrorSuppressor():
                self.mic_stream = self.audio.open(
                    format=pyaudio.paInt16,
                    channels=1, 
                    rate=self.config.sample_rate,   
                    input=True, 
                    frames_per_buffer=960,
                    input_device_index=1
                )
            logger.info('麦克风流打开完成')
        except Exception as e:
            logger.error(f"Failed to start audio stream: {e}")
            return
        
        with self.lock:
            try:
                # 初始化UDP套接字
                logger.debug('初始化UDP套接字...')
                server = self.udp_info['server']
                port = self.udp_info['port']
                logger.info(f'连接到UDP服务器: {server}:{port}')
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_socket.settimeout(1.0)
                self.udp_socket.connect((server, port))
                logger.info(f'成功连接到UDP服务器: {server}:{port}')
            except Exception as e:
                logger.error(f"Failed to connect to UDP server: {e}")
                return
        
        self.running = True
        
        # 启动音频发送线程
        logger.debug('启动音频发送线程...')
        self.send_thread = threading.Thread(target=self._send_audio, daemon=True, name="audio_send_thread")
        self.send_thread.start()
        logger.info('音频发送线程启动完成')
        
        # 启动音频接收线程
        logger.debug('启动音频接收线程...')
        self.recv_thread = threading.Thread(target=self._recv_audio, daemon=True, name="audio_recv_thread")
        self.recv_thread.start()
        logger.info('音频接收线程启动完成')
        
        logger.info("Audio service started")
    
    def stop(self):
        """停止音频服务"""
        logger.info('正在停止音频服务...')
        self.running = False
        
        # 等待发送线程结束
        if self.send_thread:
            logger.debug('等待音频发送线程结束...')
            self.send_thread.join(timeout=2.0)
            logger.info('音频发送线程已结束')
        
        # 等待接收线程结束
        if self.recv_thread:
            logger.debug('等待音频接收线程结束...')
            self.recv_thread.join(timeout=2.0)
            logger.info('音频接收线程已结束')
        
        # 关闭麦克风流
        if self.mic_stream:
            logger.debug('关闭麦克风流...')
            try:
                self.mic_stream.stop_stream()
                self.mic_stream.close()
                logger.info('麦克风流已关闭')
            except Exception as e:
                logger.error(f"Failed to close mic stream: {e}")
        
        # 关闭扬声器流
        if self.spk_stream:
            logger.debug('关闭扬声器流...')
            try:
                self.spk_stream.stop_stream()
                self.spk_stream.close()
                logger.info('扬声器流已关闭')
            except Exception as e:
                logger.error(f"Failed to close speaker stream: {e}")
        
        # 关闭PyAudio
        if self.audio:
            logger.debug('关闭PyAudio...')
            try:
                self.audio.terminate()
                logger.info('PyAudio已关闭')
            except Exception as e:
                logger.error(f"Failed to terminate PyAudio: {e}")
        
        # 关闭UDP套接字
        if self.udp_socket:
            logger.debug('关闭UDP套接字...')
            try:
                self.udp_socket.close()
                logger.info('UDP套接字已关闭')
            except Exception as e:
                logger.error(f"Failed to close UDP socket: {e}")
        
        logger.info("Audio service stopped")
    
    def restart_audio_streams(self):
        """重启音频流"""
        logger.info("正在重启音频流...")
        
        # 停止当前的运行状态
        old_running = self.running
        self.running = False
        
        # 等待现有线程结束
        if self.send_thread and self.send_thread.is_alive():
            logger.debug("等待音频发送线程结束...")
            self.send_thread.join(timeout=2.0)
        if self.recv_thread and self.recv_thread.is_alive():
            logger.debug("等待音频接收线程结束...")
            self.recv_thread.join(timeout=2.0)
        
        # 关闭麦克风流
        if self.mic_stream:
            try:
                self.mic_stream.stop_stream()
                self.mic_stream.close()
                self.mic_stream = None
                logger.debug("麦克风流已关闭")
            except Exception as e:
                logger.error(f"关闭麦克风流失败: {str(e)}")
        
        # 关闭扬声器流
        if self.spk_stream:
            try:
                self.spk_stream.stop_stream()
                self.spk_stream.close()
                self.spk_stream = None
                logger.debug("扬声器流已关闭")
            except Exception as e:
                logger.error(f"关闭扬声器流失败: {str(e)}")
        
        # 清理现有UDP连接
        if self.udp_socket:
            try:
                self.udp_socket.close()
                self.udp_socket = None
                logger.debug("UDP套接字已关闭")
            except Exception as e:
                logger.error(f"关闭UDP套接字失败: {str(e)}")
        
        try:
            # 重新初始化麦克风流
            with ALSAErrorSuppressor():
                self.mic_stream = self.audio.open(
                    format=pyaudio.paInt16,
                    channels=1, 
                    rate=self.config.sample_rate,   
                    input=True, 
                    frames_per_buffer=960,
                    input_device_index=1
                )
            logger.debug("麦克风流已重新初始化")
            
            # 创建新的UDP连接
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.settimeout(1.0)
            self.udp_socket.connect((self.udp_info['server'], self.udp_info['port']))
            logger.debug("UDP连接已重建")
            
            # 恢复运行状态
            self.running = old_running
            
            # 重置VAD检测器
            self.vad.reset()
            
            # 启动音频发送线程
            self.send_thread = threading.Thread(target=self._send_audio, daemon=True, name="audio_send_thread")
            self.send_thread.start()
            logger.debug("音频发送线程已重启")
            
            # 启动音频接收线程
            self.recv_thread = threading.Thread(target=self._recv_audio, daemon=True, name="audio_recv_thread")
            self.recv_thread.start()
            logger.debug("音频接收线程已重启")
            
            logger.info("音频流重启完成")
        except Exception as e:
            logger.error(f"重启音频流失败: {str(e)}")
            self.running = old_running
    
    def _init_speaker_stream(self):
        """初始化扬声器流，确保扬声器可用"""
        if self.spk_stream is None:
            try:
                logger.debug("初始化扬声器流")
                with ALSAErrorSuppressor():
                    # 使用48000Hz采样率，与麦克风一致
                    self.spk_stream = self.audio.open(
                        format=pyaudio.paInt16,
                        channels=1,
                        rate=48000,
                        output=True,
                        frames_per_buffer=960,
                        stream_callback=None,
                        start=True
                    )
                logger.info("扬声器流初始化完成")
                # 预填充静音数据减少延迟
                silence = b'\x00' * (960 * 2)  # 48ms静音
                self.spk_stream.write(silence)
            except Exception as e:
                logger.error(f"初始化扬声器流失败: {str(e)}")
                self.spk_stream = None
    
    def _play_voice_response(self, response_text="在呢"):
        """播放语音回应
        
        Args:
            response_text (str): 要播放的文本回应，默认为"在呢"
        """
        try:
            logger.info(f"🔊 播放语音回应: {response_text}")
            
            # 重新初始化扬声器流，确保可用
            if self.spk_stream:
                try:
                    self.spk_stream.close()
                except Exception as e:
                    logger.error(f"关闭扬声器流失败: {str(e)}")
                self.spk_stream = None
            
            # 初始化扬声器流
            self._init_speaker_stream()
            
            # 生成简单的音频信号作为回应（实际应用中应替换为TTS）
            # 生成440Hz的正弦波，持续200ms
            if self.spk_stream:
                import numpy as np
                sample_rate = 48000
                duration = 0.2  # 200ms
                frequency = 440  # 440Hz，A4音
                
                # 生成正弦波
                t = np.linspace(0, duration, int(sample_rate * duration), False)
                sine_wave = np.sin(2 * np.pi * frequency * t).astype(np.float32)
                
                # 将正弦波转换为PCM16格式
                pcm_data = (sine_wave * 32767).astype(np.int16)
                
                # 确保扬声器流正在运行
                if not self.spk_stream.is_active():
                    self.spk_stream.start_stream()
                
                # 播放音频
                self.spk_stream.write(pcm_data.tobytes())
                logger.debug("语音回应播放完成")
        except Exception as e:
            logger.error(f"播放语音回应失败: {str(e)}")
    
    def _send_audio(self):
        """音频发送线程，负责采集、编码、加密和发送音频数据"""
        nonce = self.udp_info['nonce']
        # 初始化Opus编码器
        encoder = opuslib.Encoder(self.config.sample_rate, 1, opuslib.APPLICATION_VOIP)
        frames = collections.deque()
        # 重置VAD检测器和唤醒词检测器
        self.vad.reset()
        self.wake_word_detector.reset()
        
        try:
            while self.running:
                # 读取音频数据
                # 960个样本 * 2字节/样本 = 1920字节
                # 48000Hz采样率下，960个样本 = 20ms，这是WebRTC VAD支持的帧大小
                data = self.mic_stream.read(960, exception_on_overflow=False)
                
                # 唤醒词检测 - 始终运行，即使在播放音频或处理用户语音时
                # 确保系统持续监听唤醒词，随时可以被唤醒
                wake_detected = self.wake_word_detector.feed(data)
                if wake_detected:
                    logger.info("🎉 检测到唤醒词'小智'，中断当前音频，开始录音")
                    self.is_wake_word_detected = True
                    
                    # 停止当前的音频播放（如果正在播放）
                    if self.spk_stream:
                        try:
                            self.spk_stream.stop_stream()
                            logger.debug("已停止当前音频播放")
                        except Exception as e:
                            logger.error(f"停止音频播放失败: {str(e)}")
                    
                    # 触发唤醒词回调
                    if self.on_wake_word_detected:
                        self.on_wake_word_detected()
                    
                    # 播放唤醒回应"在呢"
                    self._play_voice_response("在呢")
                    
                    # 重置VAD检测器，开始新的语音检测
                    self.vad.reset()
                    
                    # 重置VAD检测状态，确保能检测到新的语音开始
                    logger.info("🔄 重置VAD检测器，准备新的语音检测")
                    
                    # 清空之前的音频队列，开始新的录音
                    frames.clear()
                    continue
                
                # 唤醒词已检测到，进行正常的VAD检测和音频处理
                if self.is_wake_word_detected:
                    # VAD检测
                    event = self.vad.feed(data)
                    
                    if event == VadEvent.START:
                        # 检测到语音开始
                        logger.info("🎤 VAD检测到语音开始")
                        if self.on_listen_start:
                            self.on_listen_start()
                        
                        # 获取预滚缓冲区中的音频
                        pre_roll_frames = self.vad.get_pre_roll()
                        logger.debug(f"🎤 预滚缓冲区音频帧数: {len(pre_roll_frames)}")
                        for frame in pre_roll_frames:
                            frames.append(frame)
                    elif event == VadEvent.END:
                        # 检测到语音结束
                        logger.info("🎤 VAD检测到语音结束")
                        if self.on_listen_stop:
                            self.on_listen_stop()
                        # 重置唤醒词检测状态，等待下次唤醒
                        # 但保持唤醒词检测器运行，持续监听
                        logger.info("🔄 重置唤醒状态，继续监听唤醒词")
                        self.is_wake_word_detected = False
                        self.wake_word_detector.reset()
                        self.vad.reset()
                
                # 将当前帧添加到队列（仅当唤醒词已检测到）
                if self.is_wake_word_detected:
                    frames.append(data)
                    # 当队列中的帧数达到一个数据包所需的帧数时，发送数据
                    if len(frames) >= self.frames_per_packet:
                        # 取出一个数据包所需的帧数
                        packet_frames = [frames.popleft() for _ in range(min(self.frames_per_packet, len(frames)))]
                        
                        # 合并为一个数据包
                        pcm_data = b''.join(packet_frames)
                        # 使用Opus编码音频数据
                        opus_data = encoder.encode(pcm_data, 960)
                        self.local_sequence += 1
                        new_nonce = (nonce[0:4] + format(len(opus_data), '04x') +
                                    nonce[8:24] + format(self.local_sequence, '08x')) 
                        # 使用AES-CTR加密音频数据
                        encrypted_data = aes_ctr_encrypt(bytes.fromhex(self.udp_info['key']), 
                                                        bytes.fromhex(new_nonce), 
                                                        opus_data)
                        # 构建数据包
                        packet = bytes.fromhex(new_nonce) + encrypted_data
                        # 发送数据包
                        with self.lock:
                            try:
                                self.udp_socket.sendto(packet, (self.udp_info['server'], self.udp_info['port']))
                            except Exception as e:
                                if e.errno == errno.ENETUNREACH:
                                    self.restart_audio_streams()
                                    break
                                elif e.errno == errno.EBADF:  # Bad file descriptor - socket已关闭
                                    logger.info("UDP socket已关闭，停止发送")
                                    break
                                else:
                                    raise        
        except Exception as e:
            # 如果程序正在退出，只记录日志，不打印错误
            if self.running:
                logger.error(f"音频发送错误: {str(e)}")
            else:
                logger.info(f"程序退出时音频发送停止: {str(e)}")
        finally:
            if self.mic_stream is not None:
                try:
                    self.mic_stream.stop_stream()
                    self.mic_stream.close()
                except:
                    pass
    
    def _recv_audio(self):
        """音频接收线程 - 接收服务器音频并播放"""
        
        key = bytes.fromhex(self.udp_info['key'])
        sample_rate = self.audio_params['sample_rate']
        frame_duration = self.audio_params['frame_duration']
        frame_num = int(frame_duration / (1000 / sample_rate))
        # 创建Opus解码器
        decoder = opuslib.Decoder(sample_rate, 1)
        
        try:
            while self.running:
                # 确保扬声器流已初始化并正在运行
                if self.spk_stream is None:
                    # 使用指定采样率打开扬声器
                    with ALSAErrorSuppressor():
                        self.spk_stream = self.audio.open(
                            format=pyaudio.paInt16,
                            channels=1,
                            rate=sample_rate,
                            output=True,
                            frames_per_buffer=frame_num,
                            stream_callback=None, 
                            start=False
                        )
                    
                    if self.spk_stream is None:
                        logger.error("无法打开音频播放设备")
                        time.sleep(1)
                        continue
                
                try:
                    # 接收加密音频数据
                    data, _ = self.udp_socket.recvfrom(4096)
                    split_nonce = data[:16]
                    encrypt_data = data[16:]
                    # 解密音频数据
                    decrypt_data = aes_ctr_decrypt(key, split_nonce, encrypt_data)
                    
                    # 确保扬声器流正在运行
                    if not self.spk_stream.is_active():
                        # 预填充静音数据减少延迟
                        silence = b'\x00' * (frame_num * 2)
                        self.spk_stream.start_stream()
                        self.spk_stream.write(silence)
                        logger.debug("已重新启动扬声器流")
                    
                    # 播放音频
                    self.spk_stream.write(decoder.decode(decrypt_data, frame_num))
                except socket.timeout:
                    continue
                except Exception as e:
                    logger.error(f"音频接收错误: {str(e)}")
                    # 如果是流已关闭的错误，重置扬声器流
                    if "Stream closed" in str(e) or "Bad file descriptor" in str(e):
                        logger.debug("重置扬声器流")
                        try:
                            self.spk_stream.close()
                            self.spk_stream = None
                        except Exception as close_e:
                            logger.error(f"关闭扬声器流失败: {str(close_e)}")
                    time.sleep(0.1)
        except Exception as e:
            logger.error(f"播放流初始化失败: {str(e)}")
        finally:
            # 关闭扬声器流
            if self.spk_stream:
                try:
                    self.spk_stream.stop_stream()
                    self.spk_stream.close()
                    self.spk_stream = None
                except Exception as e:
                    logger.error(f"播放流关闭失败: {str(e)}")
        logger.info('AudioService recv loop exited')
