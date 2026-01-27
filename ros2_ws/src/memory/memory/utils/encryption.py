#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
加密工具模块

提供AES-256加密解密功能
"""

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import padding
import base64
import os

class Encryption:
    """加密工具类"""
    
    def __init__(self, key=None):
        """
        初始化加密工具
        
        Args:
            key (bytes, optional): 加密密钥，若不提供则生成新密钥
        """
        if key:
            self.key = key
        else:
            # 从环境变量或配置文件获取密钥
            # 这里使用默认密钥，实际应用中应从安全位置获取
            default_key = "default_memory_encryption_key_256bit"  # 32字节
            self.key = default_key.encode('utf-8')[:32]
        
        self.backend = default_backend()
    
    def encrypt(self, plaintext):
        """
        加密数据
        
        Args:
            plaintext (str or bytes): 明文数据
            
        Returns:
            str: 加密后的base64编码字符串
        """
        if isinstance(plaintext, str):
            plaintext = plaintext.encode('utf-8')
        
        # 生成随机IV
        iv = os.urandom(16)
        
        # 创建加密器
        cipher = Cipher(algorithms.AES(self.key), modes.CBC(iv), backend=self.backend)
        encryptor = cipher.encryptor()
        
        # 填充数据
        padder = padding.PKCS7(128).padder()
        padded_data = padder.update(plaintext) + padder.finalize()
        
        # 加密
        ciphertext = encryptor.update(padded_data) + encryptor.finalize()
        
        # 组合IV和密文
        encrypted = iv + ciphertext
        
        # Base64编码
        return base64.b64encode(encrypted).decode('utf-8')
    
    def decrypt(self, encrypted_text):
        """
        解密数据
        
        Args:
            encrypted_text (str): 加密后的base64编码字符串
            
        Returns:
            str: 解密后的明文
        """
        # Base64解码
        encrypted = base64.b64decode(encrypted_text)
        
        # 提取IV
        iv = encrypted[:16]
        ciphertext = encrypted[16:]
        
        # 创建解密器
        cipher = Cipher(algorithms.AES(self.key), modes.CBC(iv), backend=self.backend)
        decryptor = cipher.decryptor()
        
        # 解密
        padded_plaintext = decryptor.update(ciphertext) + decryptor.finalize()
        
        # 移除填充
        unpadder = padding.PKCS7(128).unpadder()
        plaintext = unpadder.update(padded_plaintext) + unpadder.finalize()
        
        # 转换为字符串
        return plaintext.decode('utf-8')

# 便捷函数
def aes_encrypt(text, key=None):
    """
    便捷加密函数
    
    Args:
        text (str): 要加密的文本
        key (bytes, optional): 加密密钥
        
    Returns:
        str: 加密后的文本
    """
    encryption = Encryption(key)
    return encryption.encrypt(text)

def aes_decrypt(encrypted_text, key=None):
    """
    便捷解密函数
    
    Args:
        encrypted_text (str): 加密后的文本
        key (bytes, optional): 解密密钥
        
    Returns:
        str: 解密后的文本
    """
    encryption = Encryption(key)
    return encryption.decrypt(encrypted_text)
