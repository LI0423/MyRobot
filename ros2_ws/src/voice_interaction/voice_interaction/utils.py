import os
import uuid
import glob
import logging
import requests
import time
import json
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend

def get_wlan0_ip():
    """获取wlan0网卡的IP地址
    
    Returns:
        str: wlan0网卡的IP地址，如果获取失败则返回'127.0.0.1'
    """
    try:
        import subprocess
        # 执行ip命令获取wlan0网卡信息
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'],
                                capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            # 解析命令输出，提取IP地址
            for line in result.stdout.split('\n'):
                if 'inet ' in line:
                    ip = line.strip().split()[1].split('/')[0]
                    return ip
    except Exception:
        # 获取失败时返回默认值
        pass
    return '127.0.0.1'

def get_system_mac_address():
    """获取系统的MAC地址
    
    Returns:
        str: 系统的MAC地址，优先返回wlan0网卡的MAC地址
    """
    # 尝试从wlan0网卡获取MAC地址
    try:
        wlan0_path = '/sys/class/net/wlan0/address'
        if os.path.exists(wlan0_path):
            with open(wlan0_path, 'r') as f:
                mac = f.read().strip()
                if mac and mac != '00:00:00:00:00:00':
                    return mac
    except Exception:
        pass

    # 尝试从所有非回环网卡获取MAC地址
    try:
        net_interfaces = glob.glob('/sys/class/net/*/address')
        for interface_path in net_interfaces:
            interface_name = interface_path.split('/')[-2]
            if interface_name != 'lo':  # 跳过回环接口
                try:
                    with open(interface_path, 'r') as f:
                        mac = f.read().strip()
                        if mac and mac != '00:00:00:00:00:00':
                            return mac
                except Exception:
                    continue
    except Exception:
        pass

    # 尝试使用uuid模块获取MAC地址
    try:
        mac_int = uuid.getnode()
        mac_hex = hex(mac_int)[2:].zfill(12)
        mac_formatted = ':'.join([mac_hex[i:i+2] for i in range(0, 12, 2)])
        return mac_formatted
    except Exception:
        pass

    # 所有方法都失败时返回默认MAC地址
    return '50:cf:14:5a:9f:17'

def aes_ctr_encrypt(key, nonce, plaintext):
    """AES-CTR模式加密
    
    Args:
        key (bytes): AES加密密钥，必须是16、24或32字节
        nonce (bytes): CTR模式的随机数，必须是16字节
        plaintext (bytes): 要加密的明文数据
        
    Returns:
        bytes: 加密后的密文数据
    """
    # 创建AES-CTR密码对象
    cipher = Cipher(algorithms.AES(key), modes.CTR(nonce), backend=default_backend())
    # 创建加密器
    encryptor = cipher.encryptor()
    # 执行加密并返回结果
    return encryptor.update(plaintext) + encryptor.finalize()

def aes_ctr_decrypt(key, nonce, ciphertext):
    """AES-CTR模式解密
    
    Args:
        key (bytes): AES解密密钥，必须是16、24或32字节
        nonce (bytes): CTR模式的随机数，必须与加密时使用的相同
        ciphertext (bytes): 要解密的密文数据
        
    Returns:
        bytes: 解密后的明文数据
    """
    # 创建AES-CTR密码对象
    cipher = Cipher(algorithms.AES(key), modes.CTR(nonce), backend=default_backend())
    # 创建解密器
    decryptor = cipher.decryptor()
    # 执行解密并返回结果
    plaintext = decryptor.update(ciphertext) + decryptor.finalize()
    return plaintext

def get_system_hardware_info():
    """
    获取系统实际硬件信息

    Returns:
        dict: 包含flash_size和minimum_free_heap_size的字典
            - flash_size: 系统存储大小（字节）
            - minimum_free_heap_size: 系统可用内存大小（字节）
    """
    # 初始化默认硬件信息
    hardware_info = {
        "flash_size": 16777216,        # 默认值 16MB
        "minimum_free_heap_size": 8318916  # 默认值 8MB
    }

    try:
        # 获取Flash存储信息 (eMMC/SD卡大小)
        import shutil
        total, used, free = shutil.disk_usage('/')
        # 将根分区大小作为Flash大小参考
        hardware_info["flash_size"] = total

    except Exception as e:
        logging.warning(f"无法获取Flash大小信息: {str(e)}")

    try:
        # 获取内存信息
        with open('/proc/meminfo', 'r') as f:
            meminfo = f.read()

        # 解析MemAvailable (系统可用内存)
        for line in meminfo.split('\n'):
            if line.startswith('MemAvailable:'):
                # 提取数值，单位是kB
                mem_available_kb = int(line.split()[1])
                # 转换为字节，取80%作为最小可用堆大小
                hardware_info["minimum_free_heap_size"] = int(mem_available_kb * 1024 * 0.8)
                break
        else:
            # 如果没有MemAvailable，使用MemFree
            for line in meminfo.split('\n'):
                if line.startswith('MemFree:'):
                    mem_free_kb = int(line.split()[1])
                    hardware_info["minimum_free_heap_size"] = int(mem_free_kb * 1024 * 0.8)
                    break

    except Exception as e:
        logging.warning(f"无法获取内存信息: {str(e)}")

    return hardware_info

def get_ota_version(ota_url):
    """
    从服务器获取OTA版本信息和MQTT配置
    包含设备信息上报和配置更新
    
    Args:
        ota_url (str): OTA服务器地址
        
    Returns:
        dict: 包含MQTT配置信息的字典
    """

    # 获取实际硬件信息
    hardware_info = get_system_hardware_info()

    # 构建请求头
    header = {
        'Device-Id': get_system_mac_address(),
        'Content-Type': 'application/json'
    }

    # 设备信息数据
    post_data = {
        "flash_size": hardware_info["flash_size"],
        "minimum_free_heap_size": hardware_info["minimum_free_heap_size"],
        "mac_address": get_system_mac_address(),
        "chip_model_name": "rdk",
        "chip_info": {
            "model": "RDK",
            "cores": 8,
            "revision": 1,
            "features": 32
        },
        "application": {
            "name": "xiaozhi",
            "version": "1.1.0-rdk",
            "compile_time": "Oct 15 2025",
            "idf_version": "rdk-1.1",
            "elf_sha256": "22986216df095587c42f8aeb06b239781c68ad8df80321e260556da7fcf5f522"
        },
        "partition_table": [],
        "ota": {"label": "factory"},
        "board": {
            "type": "rdk-dev",
            "ssid": "RDK-WiFi",
            "rssi": -45,
            "channel": 6,
            "ip": "192.168.1.100",
            "mac": "rdk:device:mac"
        }
    }

    try:
        # 发送POST请求获取MQTT配置
        response = requests.post(ota_url, headers=header,
                            data=json.dumps(post_data), timeout=10, verify=False)
        response.raise_for_status()
        mqtt_info = response.json()['mqtt']
        return mqtt_info
    except Exception as e:
        logging.error(f"配置更新失败: {str(e)}")
        time.sleep(5)
        # 重试获取配置
        return get_ota_version(ota_url)
