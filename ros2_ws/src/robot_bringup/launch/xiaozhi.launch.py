"""
小智语音节点启动文件

仅启动小智语音节点。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成小智节点启动描述"""
    
    xiaozhi_node = Node(
        package='llm_node',
        executable='xiaozhi_node',
        name='xiaozhi_node',
        output='screen',
        parameters=[],
    )
    
    return LaunchDescription([
        xiaozhi_node,
    ])
