"""日志配置模块

提供统一的日志配置，将日志输出到文件和控制台。
所有模块应使用 logging.getLogger(__name__) 获取logger。
"""

import logging
import os

# 日志文件路径
LOG_DIR = os.path.expanduser("~/.llm_node/logs")
LOG_FILE = os.path.join(LOG_DIR, "llm_node.log")

# 确保日志目录存在
os.makedirs(LOG_DIR, exist_ok=True)


def setup_logging(level=logging.INFO, log_file=None):
    """配置统一的日志系统
    
    Args:
        level: 日志级别，默认 INFO
        log_file: 日志文件路径，默认使用 LOG_FILE
    """
    if log_file is None:
        log_file = LOG_FILE
    
    # 日志格式
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    date_format = '%Y-%m-%d %H:%M:%S'
    
    # 创建根 logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # 清除现有的处理器
    root_logger.handlers.clear()
    
    # 创建文件处理器（写入日志文件）
    file_handler = logging.FileHandler(log_file, mode='a', encoding='utf-8')
    file_handler.setLevel(level)
    file_handler.setFormatter(logging.Formatter(log_format, date_format))
    root_logger.addHandler(file_handler)
    
    # 创建控制台处理器（输出到终端）
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(logging.Formatter(log_format, date_format))
    root_logger.addHandler(console_handler)
    
    # 记录日志系统启动
    logging.info(f"日志系统已初始化，日志文件: {log_file}")


def get_log_file_path():
    """获取当前日志文件路径
    
    Returns:
        str: 日志文件路径
    """
    return LOG_FILE
