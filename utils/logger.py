#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
日志管理工具
"""

import logging
import os
from logging.handlers import RotatingFileHandler
from datetime import datetime

# 仅在ROS2环境中导入rclpy
try:
    import rclpy
except ImportError:
    rclpy = None

class LoggerManager:
    """日志管理类"""
    
    def __init__(self):
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.ros2_loggers = {}  # 存储ROS2风格的日志记录器
        
    def setup_logger(self, log_file="robot.log", log_level=logging.INFO, max_bytes=10*1024*1024, backup_count=5):
        """配置日志记录器
        
        Args:
            log_file: 日志文件路径
            log_level: 日志级别
            max_bytes: 单个日志文件最大字节数
            backup_count: 保留的日志文件数量
            
        Returns:
            logging.Logger: 配置好的日志记录器
        """
        # 确保日志目录存在
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 清除现有的处理器
        for handler in self.logger.handlers[:]:
            self.logger.removeHandler(handler)
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(log_level)
        console_handler.setFormatter(self.formatter)
        self.logger.addHandler(console_handler)
        
        # 文件处理器
        file_handler = RotatingFileHandler(
            log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf-8'
        )
        file_handler.setLevel(log_level)
        file_handler.setFormatter(self.formatter)
        self.logger.addHandler(file_handler)
        
        self.logger.info(f"日志系统初始化完成，日志文件: {log_file}")
        return self.logger
    
    def get_logger(self, name=None):
        """获取指定名称的日志记录器
        
        Args:
            name: 日志记录器名称
            
        Returns:
            logging.Logger: 日志记录器
        """
        if name:
            return logging.getLogger(name)
        return self.logger
    
    def set_level(self, level):
        """设置日志级别
        
        Args:
            level: 日志级别
        """
        self.logger.setLevel(level)
        for handler in self.logger.handlers:
            handler.setLevel(level)
        
    def debug(self, msg, *args, **kwargs):
        """记录调试日志"""
        self.logger.debug(msg, *args, **kwargs)
    
    def info(self, msg, *args, **kwargs):
        """记录信息日志"""
        self.logger.info(msg, *args, **kwargs)
    
    def warning(self, msg, *args, **kwargs):
        """记录警告日志"""
        self.logger.warning(msg, *args, **kwargs)
    
    def error(self, msg, *args, **kwargs):
        """记录错误日志"""
        self.logger.error(msg, *args, **kwargs)
    
    def critical(self, msg, *args, **kwargs):
        """记录严重错误日志"""
        self.logger.critical(msg, *args, **kwargs)
    
    def get_ros2_logger(self, name):
        """获取ROS2风格的日志记录器
        
        Args:
            name: 日志记录器名称
            
        Returns:
            兼容ROS2日志接口的记录器
        """
        if name in self.ros2_loggers:
            return self.ros2_loggers[name]
        
        # 创建一个兼容ROS2日志接口的包装器
        class ROS2LoggerWrapper:
            def __init__(self, logger):
                self.logger = logger
            
            def debug(self, msg, *args, **kwargs):
                self.logger.debug(msg, *args, **kwargs)
            
            def info(self, msg, *args, **kwargs):
                self.logger.info(msg, *args, **kwargs)
            
            def warning(self, msg, *args, **kwargs):
                self.logger.warning(msg, *args, **kwargs)
            
            def error(self, msg, *args, **kwargs):
                self.logger.error(msg, *args, **kwargs)
            
            def critical(self, msg, *args, **kwargs):
                self.logger.critical(msg, *args, **kwargs)
        
        logger = self.get_logger(name)
        ros2_logger = ROS2LoggerWrapper(logger)
        self.ros2_loggers[name] = ros2_logger
        return ros2_logger

# 创建全局日志管理器实例
logger_manager = LoggerManager()
logger = logger_manager.get_logger()
