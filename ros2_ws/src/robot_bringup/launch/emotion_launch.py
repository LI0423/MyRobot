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

    # 语音识别节点
    voice_recognition_node = Node(
        package='emotion_interaction',
        executable='voice_recognition_node',
        name='voice_recognition',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(voice_recognition_node)

    # 情感引擎节点
    emotion_engine_node = Node(
        package='emotion_interaction',
        executable='emotion_engine_node',
        name='emotion_engine',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(emotion_engine_node)

    # 语音合成节点
    text_to_speech_node = Node(
        package='emotion_interaction',
        executable='text_to_speech_node',
        name='text_to_speech',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(text_to_speech_node)

    return ld
