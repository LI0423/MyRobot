import logging
import time
import alsaaudio


logger = logging.getLogger(__name__)


class AudioDeviceManager:
    """ALSA 音频设备管理，负责麦克风与扬声器 IO。"""

    def __init__(
        self,
        sample_rate,
        frame_samples,
        mic_device,
        spk_device,
        mic_period_size=None,
    ):
        self.sample_rate = sample_rate
        self.frame_samples = frame_samples
        self.mic_device = mic_device
        self.spk_device = spk_device
        self.mic_period_size = mic_period_size or frame_samples

        self.mic_pcm = None
        self.spk_pcm = None

    def open_mic(self):
        """打开麦克风并保持常驻。"""
        if self.mic_pcm:
            return True
        try:
            self.mic_pcm = alsaaudio.PCM(
                type=alsaaudio.PCM_CAPTURE,
                mode=alsaaudio.PCM_NORMAL,
                device=self.mic_device,
                channels=1,
                rate=self.sample_rate,
                format=alsaaudio.PCM_FORMAT_S16_LE,
                periodsize=self.mic_period_size,
                periods=4,
            )
            logger.info(f"麦克风已打开: {self.sample_rate}Hz (常驻)")
            return True
        except Exception as e:
            logger.error(f"打开麦克风失败: {e}")
            return False

    def close_mic(self):
        """关闭麦克风。"""
        if self.mic_pcm:
            try:
                self.mic_pcm.close()
            except Exception:
                pass
            self.mic_pcm = None
            logger.debug("麦克风已关闭")

    def read_mic(self):
        """读取麦克风数据，异常时自动重建设备。"""
        if not self.mic_pcm:
            return (0, None)
        try:
            return self.mic_pcm.read()
        except Exception as e:
            logger.warning(f"读取麦克风失败: {e}")
            self.close_mic()
            time.sleep(0.1)
            self.open_mic()
            return (-1, None)

    def open_spk(self, rate=16000):
        """打开扬声器。"""
        if self.spk_pcm:
            self.close_spk()
        try:
            self.spk_pcm = alsaaudio.PCM(
                type=alsaaudio.PCM_PLAYBACK,
                mode=alsaaudio.PCM_NORMAL,
                device=self.spk_device,
                channels=1,
                rate=rate,
                format=alsaaudio.PCM_FORMAT_S16_LE,
                periodsize=self.frame_samples,
            )
            logger.info(f"扬声器已打开: {rate}Hz")
            return True
        except Exception as e:
            logger.error(f"打开扬声器失败: {e}")
            return False

    def close_spk(self):
        """关闭扬声器。"""
        if self.spk_pcm:
            try:
                self.spk_pcm.close()
            except Exception:
                pass
            self.spk_pcm = None
            logger.debug("扬声器已关闭")

    def write_spk(self, pcm_data):
        """写入扬声器。"""
        if not self.spk_pcm:
            return False
        self.spk_pcm.write(pcm_data)
        return True

    def close_all(self):
        """关闭全部设备。"""
        self.close_mic()
        self.close_spk()
