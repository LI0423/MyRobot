import threading
import socket
import time
import opuslib
import os
import logging
from enum import Enum
from llm_node.config import Config
from llm_node.vad_detector import VADDetector
from llm_node.integrated_wake_detector import IntegratedWakeWordDetector
from llm_node.audio_device_manager import AudioDeviceManager
from llm_node.udp_audio_transport import UdpAudioTransport
from llm_node.recorder_session import RecorderSession


logger = logging.getLogger(__name__)


class AudioState(Enum):
    """音频服务状态枚举"""
    IDLE = "idle"                    # 唤醒词监听中
    RESPONDING = "responding"        # 播放唤醒回应中
    RECORDING = "recording"          # 录音中
    UPLOADING = "uploading"          # 上传语音中
    WAITING = "waiting"              # 等待服务器响应
    PLAYING_TTS = "playing_tts"      # 播放TTS回复中


class AudioService:
    """音频服务类 - 简化版架构
    
    核心特点:
    - 麦克风始终保持打开 (16kHz)，避免设备状态问题
    - 原生 16kHz 录音，无需降采样，提高唤醒准确率和上传音质
    - 状态机控制流程，但不频繁开关设备
    """

    def __init__(self, config: Config, on_listen_start=None, on_listen_stop=None, on_wake_word_detected=None):
        """初始化音频服务"""
        self.config = config
        self.on_listen_start = on_listen_start
        self.on_listen_stop = on_listen_stop
        self.on_wake_word_detected = on_wake_word_detected

        # ========== 状态机 ==========
        self._state = AudioState.IDLE
        self._state_lock = threading.Lock()
        
        # ========== 常量定义 (消除硬编码) ==========
        # 帧时长: 60ms
        self.frame_duration_ms = config.frame_duration
        # 帧样本数: 16000 * 0.06 = 960
        self.frame_samples = int(self.config.sample_rate * self.frame_duration_ms / 1000)
        # 帧字节数: 960 * 2 (16-bit) = 1920
        self.frame_bytes = self.frame_samples * 2
        
        # ========== 录音会话配置 ==========
        self._max_recording_sec = 30
        self._max_buffer_frames = int(self._max_recording_sec * config.sample_rate / self.frame_samples)
        self._no_speech_timeout = 10
        
        # ========== 等待状态超时 ==========
        self._waiting_start_time = 0
        self._waiting_timeout = 30  # 等待服务器响应的最大时间（秒）
        
        # ========== 唤醒词检测器 (16kHz) ==========
        self.wake_word_detector = IntegratedWakeWordDetector(
            model_dir=config.sherpa_model_dir,
            keywords_file=config.sherpa_keywords_file,
            sample_rate=config.sample_rate
        )
        
        # ========== VAD 检测器 (16kHz) ==========
        self.vad = VADDetector(
            sample_rate=config.sample_rate,
            frame_ms=config.vad_frame_ms,
            mode=config.vad_mode,
            start_frames=config.vad_start_frames,
            end_silence_ms=config.vad_end_silence_ms,
            pre_roll_ms=config.vad_pre_roll_ms
        )
        self.recorder = RecorderSession(
            vad=self.vad,
            sample_rate=config.sample_rate,
            vad_frame_ms=config.vad_frame_ms,
            max_buffer_frames=self._max_buffer_frames,
            no_speech_timeout=self._no_speech_timeout,
            pre_roll_frames=15
        )
        
        # ========== ALSA 设备 ==========
        # 60ms @ 16kHz = 960样本
        # VAD分割 = 3个 20ms 帧 (3 * 320 = 960) - 完美匹配
        self.device_manager = AudioDeviceManager(
            sample_rate=self.config.sample_rate,
            frame_samples=self.frame_samples,
            mic_device=self.config.mic_device,
            spk_device=self.config.spk_device,
            mic_period_size=self.frame_samples
        )
        
        # ========== 网络 ==========
        self.lock = threading.Lock()
        self.audio_params = config.aes_opus_info['audio_params']
        self.transport = UdpAudioTransport(config.aes_opus_info['udp'])
        
        # ========== Opus 编解码器 ==========
        self._encoder = None
        self._decoder = None
        
        # ========== 线程 ==========
        self._main_thread = None
        self._recv_thread = None
        self.running = False
        
        # ========== 会话 ==========
        self.session_id = None
    

    
    @property
    def state(self):
        """获取当前状态"""
        with self._state_lock:
            return self._state
    
    def _set_state(self, new_state: AudioState):
        """设置新状态"""
        with self._state_lock:
            old_state = self._state
            self._state = new_state
            logger.info(f"状态转换: {old_state.value} → {new_state.value}")

    def _set_state_if(self, expected_state: AudioState, new_state: AudioState):
        """条件状态迁移: 仅当当前状态等于 expected_state 时才迁移。"""
        with self._state_lock:
            if self._state != expected_state:
                return False
            old_state = self._state
            self._state = new_state
            logger.info(f"状态转换: {old_state.value} → {new_state.value}")
            return True
    
    def update_udp_info(self, udp_dict):
        """更新UDP配置信息"""
        self.transport.update_udp_info(udp_dict)
    
    def update_audio_params(self, audio_params_dict):
        """更新音频参数信息（用于 TTS 播放）"""
        with self.lock:
            requested_sample_rate = audio_params_dict.get('sample_rate')
            current_sample_rate = self.audio_params.get('sample_rate')

            # 运行中采样率切换时，同步重建 decoder，保证参数与解码器一致
            if (
                requested_sample_rate is not None
                and requested_sample_rate != current_sample_rate
                and self._decoder is not None
            ):
                try:
                    self._decoder = opuslib.Decoder(requested_sample_rate, 1)
                    logger.info(f"Decoder 已重建: sample_rate={requested_sample_rate}")
                except Exception as e:
                    logger.error(f"重建 Decoder 失败，放弃本次音频参数更新: {e}")
                    return

            self.audio_params.update(audio_params_dict)
            logger.info(f"音频参数已更新: sample_rate={self.audio_params.get('sample_rate')}")
    
    # ========== 服务启动/停止 ==========
    
    def start(self):
        """启动音频服务"""
        logger.info('启动音频服务...')
        if self.running:
            return
        
        # 初始化 UDP (不使用 connect()，因为服务器可能从不同端口响应)
        try:
            self.transport.create_socket(timeout=1.0, reset_sequence=True)
            server, port = self.transport.target()
            logger.info(f"UDP socket 已创建，目标: {server}:{port}")
        except Exception as e:
            logger.error(f"UDP 初始化失败: {e}")
            return
        
        # 打开麦克风 (始终保持打开)
        if not self.device_manager.open_mic():
            logger.error("无法打开麦克风，服务启动失败")
            self.transport.close_socket()
            return
        
        # 初始化编解码器
        self._encoder = opuslib.Encoder(self.config.sample_rate, 1, opuslib.APPLICATION_AUDIO)
        self._decoder = opuslib.Decoder(self.audio_params['sample_rate'], 1)
        
        self.running = True
        self._set_state(AudioState.IDLE)
        
        # 启动主线程 (状态机循环)
        self._main_thread = threading.Thread(target=self._main_loop, daemon=True, name="audio_main")
        self._main_thread.start()
        
        # 启动接收线程
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True, name="audio_recv")
        self._recv_thread.start()
        
        logger.info("音频服务已启动")
    
    def stop(self):
        """停止音频服务"""
        logger.info('停止音频服务...')
        self.running = False
        
        if self._main_thread:
            self._main_thread.join(timeout=2.0)
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        
        self.device_manager.close_all()
        
        self.transport.close_socket()
        
        logger.info("音频服务已停止")
    
    def restart_audio_streams(self):
        """重启音频流 (UDP 重连)
        
        不使用 connect()，因为服务器可能从不同端口响应
        """
        logger.info("重启音频流...")
        try:
            self.transport.create_socket(timeout=1.0, reset_sequence=False)
            server, port = self.transport.target()
            logger.info(f"UDP socket 已创建，目标: {server}:{port}")
        except Exception as e:
            logger.error(f"UDP 重连失败: {e}")
    
    # ========== 状态机主循环 ==========
    
    def _main_loop(self):
        """状态机主循环"""
        logger.info("状态机主循环启动")
        
        while self.running:
            current_state = self.state
            
            if current_state == AudioState.IDLE:
                self._handle_idle_state()
            elif current_state == AudioState.RESPONDING:
                self._handle_responding_state()
            elif current_state == AudioState.RECORDING:
                # 默认使用批量模式
                # 如需流式上传，请在此处改为 self._handle_recording_state_streaming()
                self._handle_recording_state()
            elif current_state == AudioState.UPLOADING:
                self._handle_uploading_state()
            elif current_state == AudioState.WAITING:
                self._handle_waiting_state()
            elif current_state == AudioState.PLAYING_TTS:
                self._handle_playing_tts_state()
            else:
                time.sleep(0.1)
        
        logger.info("状态机主循环退出")
    
    def _handle_idle_state(self):
        """IDLE 状态: 唤醒词监听
        
        麦克风始终保持打开 (48kHz)，读取后降采样给唤醒词检测器
        """
        length, data = self.device_manager.read_mic()
        if length <= 0:
            if length < 0:
                time.sleep(0.1)
            return
        
        # 唤醒词检测 (直接使用 16kHz 原生数据)
        try:
            wake_detected = self.wake_word_detector.feed_audio(data)
            if wake_detected:
                logger.info("检测到唤醒词!")
                
                # 触发回调
                if self.on_wake_word_detected:
                    self.on_wake_word_detected()
                
                self._set_state(AudioState.RESPONDING)
        except Exception as e:
            logger.error(f"唤醒词检测错误: {e}")
    
    def _handle_responding_state(self):
        """RESPONDING 状态: 播放唤醒回应
        
        注意：麦克风保持打开，但数据会被丢弃（避免录到回放）
        """
        logger.info("播放唤醒回应...")
        
        # 打开扬声器
        if not self.device_manager.open_spk(rate=self.config.sample_rate):
            self.recorder.reset_for_new_recording()
            self._set_state(AudioState.RECORDING)
            return

        # 增加等待时间，让回声消散
        time.sleep(0.3)
        
        # 15 * 60ms = 900ms，足以覆盖回放时间和回声
        for _ in range(15):
            self.device_manager.read_mic()
        
        # 进入录音状态
        self.recorder.reset_for_new_recording()
        self._set_state(AudioState.RECORDING)
        logger.info("开始等待用户说话...")
    
    def _send_audio_frame(self, frame_data):
        """发送单帧音频 (16kHz, 60ms)"""
        try:
            # 1. 检查帧大小 (16kHz * 60ms * 2bytes = 1920 bytes)
            if len(frame_data) != self.frame_bytes:
                return
            
            # 2. Opus 编码
            opus_data = self._encoder.encode(frame_data, self.frame_samples)
            
            # 3. 交给传输层执行加密和 UDP 发送
            self.transport.send_opus(opus_data)
            
        except Exception as e:
            logger.error(f"发送音频帧失败: {e}")
    
    def _handle_recording_state(self):
        """RECORDING 状态: 录音 (默认 - 批量模式)
        
        麦克风读取 60ms 帧 (960 样本 @ 16kHz)
        缓冲直到录音结束，然后转入 UPLOADING 状态
        """
        length, data = self.device_manager.read_mic()
        if length <= 0:
            if length < 0:
                time.sleep(0.1)
            return
        
        result = self.recorder.process_frame(data)

        if result["listen_started"]:
            logger.info("VAD 检测到语音开始")
            if self.on_listen_start:
                self.on_listen_start()

        if result["end_detected"]:
            logger.info(f"录音结束，共 {self.recorder.frame_count()} 帧")
            self._set_state(AudioState.UPLOADING)
        elif result["no_speech_timeout"]:
            logger.warning(f"等待语音超时 ({self._no_speech_timeout}秒)，返回 IDLE")
            self._set_state(AudioState.IDLE)
        elif result["max_duration"]:
            logger.warning("录音超时，强制结束")
            self._set_state(AudioState.UPLOADING)

    def _handle_recording_state_streaming(self):
        """RECORDING 状态: 录音 (可选 - 流式模式)
        
        开启方法：在 _main_loop 中将 _handle_recording_state 替换为此方法
        特点：实时发送录音帧，延迟更低
        """
        length, data = self.device_manager.read_mic()
        if length <= 0:
            if length < 0:
                time.sleep(0.1)
            return

        result = self.recorder.process_frame(data)

        if result["listen_started"]:
            logger.info("VAD 检测到语音开始 (Streaming模式)")
            if self.on_listen_start:
                self.on_listen_start()

        for frame in result["new_frames"]:
            self._send_audio_frame(frame)

        if result["end_detected"]:
            logger.info(f"录音结束 (Streaming完成)，共 {self.recorder.frame_count()} 帧")
            if self.on_listen_stop:
                self.on_listen_stop()
            self._set_state(AudioState.WAITING)
        elif result["no_speech_timeout"]:
            logger.warning(f"等待语音超时 ({self._no_speech_timeout}秒)，返回 IDLE")
            self._set_state(AudioState.IDLE)
        elif result["max_duration"]:
            logger.warning("录音超时，强制结束")
            if self.on_listen_stop:
                self.on_listen_stop()
            self._set_state(AudioState.WAITING)
    
    def _handle_uploading_state(self):
        """UPLOADING 状态: 批量上传录音 (批量模式专用)"""
        frame_list = self.recorder.pop_audio_buffer()
        logger.info(f"开始上传 {len(frame_list)} 帧音频...")
        
        if not frame_list:
            logger.warning("录音缓冲区为空")
            self._waiting_start_time = time.time()
            self._set_state(AudioState.WAITING)
            return
        
        try:
            for frame in frame_list:
                # 复用发送逻辑，但需要加上时序控制
                self._send_audio_frame(frame)
                
                # 模拟实时发送 (60ms 一帧)
                time.sleep(self.frame_duration_ms / 1000.0)
            
            logger.info("音频上传完成")
            
            # 上传完成后，告诉服务器"我说完了"
            if self.on_listen_stop:
                self.on_listen_stop()
                
        except Exception as e:
            logger.error(f"上传失败: {e}")
        
        self._waiting_start_time = time.time()
        self._set_state(AudioState.WAITING)
    
    def _handle_waiting_state(self):
        """WAITING 状态: 等待服务器响应
        
        继续读取麦克风（丢弃数据），保持设备活跃
        如果超时则返回 IDLE
        """
        self.device_manager.read_mic()  # 读取并丢弃
        
        # 检查等待超时
        if time.time() - self._waiting_start_time > self._waiting_timeout:
            logger.warning(f"等待服务器响应超时 ({self._waiting_timeout}秒)，返回 IDLE")
            self.wake_word_detector.reset()
            self._set_state_if(AudioState.WAITING, AudioState.IDLE)
            return
        
        time.sleep(0.01)
    
    def _handle_playing_tts_state(self):
        """PLAYING_TTS 状态: 播放 TTS 回复
        
        继续读取麦克风（丢弃数据），检测打断
        """
        length, data = self.device_manager.read_mic()
        if length > 0:
            # 检测唤醒词（可用于打断）
            if self.wake_word_detector.feed_audio(data):
                logger.info("TTS 播放被唤醒词打断")
                self.device_manager.close_spk()
                self.wake_word_detector.reset()
                self._set_state(AudioState.RESPONDING)
        time.sleep(0.01)
    
    # ========== 接收线程 ==========
    
    def _recv_loop(self):
        """音频接收线程"""
        logger.info("接收线程启动")
        
        tts_buffer = []  # 用于存储接收到的 TTS 音频数据
        
        while self.running:
            current_state = self.state
            if current_state not in (AudioState.WAITING, AudioState.PLAYING_TTS):
                time.sleep(0.05)
                continue
            
            # 每次循环读取最新的 UDP 配置（可能被 HELLO 响应更新）
            try:
                with self.lock:
                    sample_rate = self.audio_params['sample_rate']
                    frame_duration = self.audio_params['frame_duration']
                    decoder = self._decoder
                frame_num = int(frame_duration / (1000 / sample_rate))
                if decoder is None:
                    logger.warning("Decoder 未初始化，跳过本次接收帧")
                    time.sleep(0.05)
                    continue
            except Exception as e:
                logger.error(f"读取 UDP 配置失败: {e}")
                time.sleep(0.1)
                continue
            
            try:
                decrypt_data, addr = self.transport.recv_decrypted(bufsize=4096)
                logger.debug(f"收到 UDP 数据，来自 {addr}")
                
                if current_state == AudioState.WAITING:
                    switched = self._set_state_if(AudioState.WAITING, AudioState.PLAYING_TTS)
                    if not switched:
                        logger.debug("收到 UDP 包但状态已变化，丢弃该包")
                        continue

                    logger.info("收到服务器响应，开始播放 TTS")
                    # 播放长音频时，增加超时时间以容忍网络抖动
                    self.transport.set_timeout(2.0)
                    tts_buffer = []  # 开始新的 TTS 播放，清空缓冲区
                    if not self.device_manager.open_spk(rate=sample_rate):
                        self._set_state_if(AudioState.PLAYING_TTS, AudioState.WAITING)
                        continue
                
                # 解码
                pcm_data = decoder.decode(decrypt_data, frame_num)
                
                self.device_manager.write_spk(pcm_data)
                
                # 收集音频数据用于保存
                tts_buffer.append(pcm_data)
                    
            except socket.timeout:
                if self.state == AudioState.PLAYING_TTS:
                    logger.info("TTS 播放完成")
                    
                    # 恢复默认超时
                    try:
                        self.transport.set_timeout(1.0)
                    except:
                        pass

                    tts_buffer = []
                    self.device_manager.close_spk()
                    self.wake_word_detector.reset()
                    self._set_state(AudioState.IDLE)
                elif self.state == AudioState.WAITING:
                    # WAITING 状态下超时，继续等待
                    logger.debug("等待服务器响应...")
            except Exception as e:
                logger.error(f"接收错误: {e}")
                time.sleep(0.1)
        
        logger.info("接收线程退出")

    # ========== 外部控制接口 ==========
    
    def force_idle(self):
        """强制回到 IDLE 状态"""
        logger.info("强制回到 IDLE 状态")
        self.device_manager.close_spk()
        self.recorder.clear()
        self.wake_word_detector.reset()
        self._set_state(AudioState.IDLE)
        # 注意：不关闭麦克风，保持常开
    
    def is_in_conversation(self):
        """是否在对话中"""
        return self.state not in (AudioState.IDLE,)

    def play_response(self, text):
        """播放语音回复 (兼容旧接口)"""
        logger.info(f"播放回复请求: {text}")
