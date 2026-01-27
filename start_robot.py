#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人启动脚本
统一启动入口，支持多种启动方式
"""

import argparse
import subprocess
import sys
import os

# 添加项目根目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入工具模块
from utils.logger import logger_manager

class RobotStarter:
    """机器人启动器"""
    
    def __init__(self):
        self.logger = logger_manager.get_ros2_logger("机器人启动器")
        self.logger.info("初始化机器人启动器...")
    
    def parse_arguments(self):
        """解析命令行参数"""
        parser = argparse.ArgumentParser(description="小陪智能陪伴机器人启动脚本")
        parser.add_argument('--mode', '-m', choices=['all', 'vision', 'life', 'monitor', 'entertainment', 'home', 'xiaozhi'], 
                            default='all', help="启动模式")
        parser.add_argument('--disable-vision', action='store_true', help="禁用视觉感知模块")
        parser.add_argument('--disable-life', action='store_true', help="禁用生活辅助模块")
        parser.add_argument('--disable-monitor', action='store_true', help="禁用远程监控模块")
        parser.add_argument('--disable-entertainment', action='store_true', help="禁用娱乐互动模块")
        parser.add_argument('--disable-home', action='store_true', help="禁用家居控制模块")
        parser.add_argument('--disable-voice', action='store_true', help="禁用语音交互模块")
        parser.add_argument('--sim-time', action='store_true', help="使用仿真时间")
        return parser.parse_args()
    
    def generate_launch_command(self, args):
        """生成ROS2启动命令"""
        ros2_ws_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_ws")
        
        if args.mode == 'all':
            # 启动所有模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                f"enable_vision:={str(not args.disable_vision).lower()}",
                f"enable_life:={str(not args.disable_life).lower()}",
                f"enable_monitor:={str(not args.disable_monitor).lower()}",
                f"enable_entertainment:={str(not args.disable_entertainment).lower()}",
                f"enable_home:={str(not args.disable_home).lower()}",
                f"enable_voice:={str(not args.disable_voice).lower()}"
            ]
        elif args.mode == 'vision':
            # 只启动视觉感知模块
            cmd = [
                "ros2", "launch", "robot_bringup", "vision_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}"
            ]
        elif args.mode == 'xiaozhi':
            # 只启动小智语音助手模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                "enable_vision:=false",
                "enable_life:=false",
                "enable_monitor:=false",
                "enable_entertainment:=false",
                "enable_home:=false",
                "enable_voice:=true"
            ]
        elif args.mode == 'life':
            # 只启动生活辅助模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                "enable_vision:=false",
                "enable_life:=true",
                "enable_monitor:=false",
                "enable_entertainment:=false",
                "enable_home:=false",
                "enable_voice:={str(not args.disable_voice).lower()}"
            ]
        elif args.mode == 'monitor':
            # 只启动远程监控模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                "enable_vision:=false",
                "enable_life:=false",
                "enable_monitor:=true",
                "enable_entertainment:=false",
                "enable_home:=false",
                "enable_voice:={str(not args.disable_voice).lower()}"
            ]
        elif args.mode == 'entertainment':
            # 只启动娱乐互动模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                "enable_vision:=false",
                "enable_life:=false",
                "enable_monitor:=false",
                "enable_entertainment:=true",
                "enable_home:=false",
                "enable_voice:={str(not args.disable_voice).lower()}"
            ]
        elif args.mode == 'home':
            # 只启动家居控制模块
            cmd = [
                "ros2", "launch", "robot_bringup", "robot_launch.py",
                f"use_sim_time:={str(args.sim_time).lower()}",
                "enable_vision:=false",
                "enable_life:=false",
                "enable_monitor:=false",
                "enable_entertainment:=false",
                "enable_home:=true",
                "enable_voice:={str(not args.disable_voice).lower()}"
            ]
        else:
            self.logger.error(f"未知启动模式: {args.mode}")
            sys.exit(1)
        
        return cmd
    
    def start(self):
        """启动机器人"""
        # 解析命令行参数
        args = self.parse_arguments()
        
        # 生成启动命令
        cmd = self.generate_launch_command(args)
        
        self.logger.info(f"准备启动机器人，命令: {' '.join(cmd)}")
        
        # 检查ROS2是否可用
        try:
            subprocess.run(["ros2", "--version"], check=True, capture_output=True, text=True)
            self.logger.info("ROS2环境检测成功")
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.logger.error("ROS2环境检测失败，请确保ROS2已正确安装并配置环境变量")
            sys.exit(1)
        
        # 切换到ROS2工作空间目录
        ros2_ws_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_ws")
        os.chdir(ros2_ws_path)
        
        # 执行启动命令
        try:
            self.logger.info("启动机器人系统...")
            self.logger.info("按Ctrl+C停止机器人")
            subprocess.run(cmd, check=True)
        except KeyboardInterrupt:
            self.logger.info("收到中断信号，机器人正在停止...")
        except subprocess.CalledProcessError as e:
            self.logger.error(f"启动失败: {e}")
            sys.exit(1)
        except Exception as e:
            self.logger.error(f"启动过程中发生错误: {e}")
            sys.exit(1)
        finally:
            self.logger.info("机器人已停止")


if __name__ == "__main__":
    starter = RobotStarter()
    starter.start()
