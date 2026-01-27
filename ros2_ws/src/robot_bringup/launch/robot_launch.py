#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_vision = LaunchConfiguration('enable_vision', default='true')
    enable_life = LaunchConfiguration('enable_life', default='true')
    enable_monitor = LaunchConfiguration('enable_monitor', default='true')
    enable_entertainment = LaunchConfiguration('enable_entertainment', default='true')
    enable_home = LaunchConfiguration('enable_home', default='true')
    enable_voice = LaunchConfiguration('enable_voice', default='true')

    # 创建启动描述
    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value=use_sim_time))
    ld.add_action(DeclareLaunchArgument('enable_vision', default_value=enable_vision))
    ld.add_action(DeclareLaunchArgument('enable_life', default_value=enable_life))
    ld.add_action(DeclareLaunchArgument('enable_monitor', default_value=enable_monitor))
    ld.add_action(DeclareLaunchArgument('enable_entertainment', default_value=enable_entertainment))
    ld.add_action(DeclareLaunchArgument('enable_home', default_value=enable_home))
    ld.add_action(DeclareLaunchArgument('enable_voice', default_value=enable_voice))

    # 启动小智语音助手模块 - 优先启动
    xiaozhi_launch = IncludeLaunchDescription(
        '/Users/litengjiang/Desktop/MyCode/MyRobot/ros2_ws/src/xiaozhi_voice/launch/xiaozhi_voice_launch.py',
        condition=IfCondition(enable_voice),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(xiaozhi_launch)

    # 启动视觉感知模块
    vision_launch = IncludeLaunchDescription(
        '/Users/litengjiang/Desktop/MyCode/MyRobot/ros2_ws/src/robot_bringup/launch/vision_launch.py',
        condition=IfCondition(enable_vision),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(vision_launch)
    
    # 启动生活辅助模块
    life_node = Node(
        package='life_assistance',
        executable='schedule_reminder_node',
        name='schedule_reminder',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_life)
    )
    ld.add_action(life_node)
    
    # 启动远程监控模块
    monitor_node = Node(
        package='remote_monitoring',
        executable='video_streaming_node',
        name='video_streaming',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_monitor)
    )
    ld.add_action(monitor_node)
    
    # 启动娱乐互动模块
    entertainment_node = Node(
        package='entertainment',
        executable='story_player_node',
        name='story_player',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_entertainment)
    )
    ld.add_action(entertainment_node)
    
    # 启动家居控制模块
    home_node = Node(
        package='home_control',
        executable='home_control_node',
        name='home_control',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_home)
    )
    ld.add_action(home_node)

    return ld
