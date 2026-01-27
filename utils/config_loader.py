#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
配置文件加载工具
"""

import json
import logging
import os
from pathlib import Path

# 仅在ROS2环境中导入rclpy
try:
    import rclpy
except ImportError:
    rclpy = None

logger = logging.getLogger(__name__)

class ConfigLoader:
    """配置文件加载类"""
    
    def __init__(self, config_path=None):
        # 支持多种路径方式，确保在ROS2环境中也能找到配置文件
        if config_path is None:
            # 尝试多种可能的配置文件路径
            possible_paths = [
                "config/config.json",  # 原始路径
                "/config/config.json",  # 绝对路径
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/config.json"),  # 相对于工具模块的路径
                "ros2_ws/src/config/config.json"  # ROS2工作空间内的路径
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    config_path = path
                    break
            else:
                config_path = "config/config.json"  # 默认路径
        
        self.config_path = config_path
        self.config = {}
        self.ros2_node = None  # ROS2节点引用，用于参数同步
        
    def set_ros2_node(self, node):
        """设置ROS2节点引用，用于参数同步"""
        self.ros2_node = node
        
    def load_config(self):
        """加载配置文件"""
        try:
            if not os.path.exists(self.config_path):
                logger.error(f"配置文件不存在: {self.config_path}")
                return False
            
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
            
            logger.info(f"配置文件加载成功: {self.config_path}")
            
            # 如果有ROS2节点，将配置同步到参数服务器
            if self.ros2_node:
                self._sync_to_ros2_params()
            
            return True
            
        except json.JSONDecodeError as e:
            logger.error(f"配置文件解析错误: {str(e)}")
            return False
        except Exception as e:
            logger.error(f"加载配置文件时发生错误: {str(e)}")
            return False
    
    def _sync_to_ros2_params(self):
        """将配置同步到ROS2参数服务器"""
        if not self.ros2_node or not self.config or rclpy is None:
            return
        
        try:
            # 递归同步配置到ROS2参数服务器
            def sync_dict(params_dict, prefix=""):
                for key, value in params_dict.items():
                    param_name = f"{prefix}{key}" if prefix else key
                    if isinstance(value, dict):
                        sync_dict(value, f"{param_name}.")
                    else:
                        self.ros2_node.set_parameter(
                            rclpy.parameter.Parameter(
                                param_name,
                                rclpy.parameter.Parameter.Type.from_python_type(value),
                                value
                            )
                        )
            
            sync_dict(self.config)
            logger.info("配置已同步到ROS2参数服务器")
        except Exception as e:
            logger.error(f"同步配置到ROS2参数服务器失败: {str(e)}")
    
    def get_config(self, key=None, default=None):
        """获取配置项
        
        Args:
            key: 配置项键名，支持点分隔符（如 "hardware.camera.enabled"）
            default: 默认值
            
        Returns:
            配置值或默认值
        """
        if not self.config:
            self.load_config()
        
        if not key:
            return self.config
        
        keys = key.split('.')
        value = self.config
        
        try:
            for k in keys:
                value = value[k]
            return value
        except KeyError:
            logger.warning(f"配置项不存在: {key}")
            return default
        except TypeError:
            logger.warning(f"配置项访问错误: {key}")
            return default
    
    def save_config(self, new_config=None):
        """保存配置文件
        
        Args:
            new_config: 新的配置内容，如果为None则保存当前配置
            
        Returns:
            bool: 保存成功返回True，否则返回False
        """
        try:
            config_to_save = new_config if new_config else self.config
            
            # 确保配置目录存在
            config_dir = os.path.dirname(self.config_path)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)
            
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(config_to_save, f, indent=2, ensure_ascii=False)
            
            logger.info(f"配置文件保存成功: {self.config_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存配置文件时发生错误: {str(e)}")
            return False
    
    def update_config(self, key, value):
        """更新配置项
        
        Args:
            key: 配置项键名，支持点分隔符
            value: 新的配置值
            
        Returns:
            bool: 更新成功返回True，否则返回False
        """
        if not self.config:
            self.load_config()
        
        keys = key.split('.')
        config = self.config
        
        try:
            # 遍历到最后一个键的父级
            for k in keys[:-1]:
                if k not in config:
                    config[k] = {}
                config = config[k]
            
            # 更新值
            config[keys[-1]] = value
            logger.info(f"配置项更新成功: {key} = {value}")
            return self.save_config()
            
        except Exception as e:
            logger.error(f"更新配置项时发生错误: {str(e)}")
            return False

# 创建全局配置加载器实例
config_loader = ConfigLoader()
