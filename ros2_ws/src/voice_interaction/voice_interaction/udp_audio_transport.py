import logging
import socket
import threading

from llm_node.utils import aes_ctr_encrypt, aes_ctr_decrypt


logger = logging.getLogger(__name__)


class UdpAudioTransport:
    """UDP 音频传输层，负责 socket 生命周期与加解密收发。"""

    def __init__(self, udp_info):
        self._lock = threading.Lock()
        self._udp_info = dict(udp_info)
        self._socket = None
        self._local_sequence = 0

    def update_udp_info(self, udp_dict):
        with self._lock:
            self._udp_info.update(udp_dict)

    def create_socket(self, timeout=1.0, reset_sequence=False):
        self.close_socket()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(timeout)
        if reset_sequence:
            self._local_sequence = 0

    def close_socket(self):
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    def set_timeout(self, timeout):
        if self._socket:
            self._socket.settimeout(timeout)

    def target(self):
        with self._lock:
            return self._udp_info["server"], self._udp_info["port"]

    def send_opus(self, opus_data):
        if not self._socket:
            raise RuntimeError("UDP socket is not initialized")

        with self._lock:
            nonce = self._udp_info["nonce"]
            key = bytes.fromhex(self._udp_info["key"])
            server = self._udp_info["server"]
            port = self._udp_info["port"]
            self._local_sequence += 1
            seq = self._local_sequence

        new_nonce = nonce[0:4] + format(len(opus_data), "04x") + nonce[8:24] + format(seq, "08x")
        encrypted_data = aes_ctr_encrypt(key, bytes.fromhex(new_nonce), opus_data)
        packet = bytes.fromhex(new_nonce) + encrypted_data
        self._socket.sendto(packet, (server, port))

    def recv_decrypted(self, bufsize=4096):
        if not self._socket:
            raise RuntimeError("UDP socket is not initialized")

        data, addr = self._socket.recvfrom(bufsize)
        if len(data) < 16:
            raise ValueError("UDP packet too short")

        split_nonce = data[:16]
        encrypt_data = data[16:]
        with self._lock:
            key = bytes.fromhex(self._udp_info["key"])
        decrypt_data = aes_ctr_decrypt(key, split_nonce, encrypt_data)
        return decrypt_data, addr
