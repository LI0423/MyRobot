#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'
        ),
        
        # 启动小智助手节点
        Node(
            package='voice_interaction',
            executable='xiaozhi_assistant_node',
            name='xiaozhi_assistant_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])