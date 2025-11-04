#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC Voice Control Launch File
WHEELTEC 语音控制启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('largemodel_wheeltec')
    params_file = os.path.join(pkg_share, 'config', 'wheeltec_config.yaml')

    # 启动参数
    text_chat_mode = LaunchConfiguration('text_chat_mode', default='false')
    text_chat_mode_arg = DeclareLaunchArgument(
        'text_chat_mode',
        default_value=text_chat_mode,
        description='Enable text chat mode (disable voice): true/false'
    )

    # Action Service 节点
    action_server = Node(
        package='largemodel_wheeltec',
        executable='action_service',
        name='wheeltec_action_service',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        text_chat_mode_arg,
        action_server,
    ])


if __name__ == '__main__':
    generate_launch_description()
