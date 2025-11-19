#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC Text Control Launch File
WHEELTEC 文本控制启动文件

This launch file starts both the action service and text chat interface
此启动文件同时启动动作服务和文本输入界面
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述 / Generate launch description"""
    
    # 获取配置文件路径 / Get config file path
    pkg_share = get_package_share_directory('largemodel_wheeltec')
    params_file = os.path.join(pkg_share, 'config', 'wheeltec_config.yaml')

    # Action Service 节点 / Action Service node
    action_server = Node(
        package='largemodel_wheeltec',
        executable='action_service',
        name='wheeltec_action_service',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Text Chat 节点 / Text Chat node
    text_chat = Node(
        package='largemodel_wheeltec',
        executable='text_chat',
        name='wheeltec_text_chat',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        action_server,
        text_chat,
    ])


if __name__ == '__main__':
    generate_launch_description()
