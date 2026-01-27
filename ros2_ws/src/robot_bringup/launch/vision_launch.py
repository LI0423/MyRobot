#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 创建启动描述
    ld = LaunchDescription()

    # 人脸识别节点
    face_detection_node = Node(
        package='vision_perception',
        executable='face_detection_node',
        name='face_detection',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(face_detection_node)

    return ld
