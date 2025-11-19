#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC Robot Text Chat Interface
WHEELTEC 机器人文本输入界面
Adapted from ROSMASTER M3 text_chat
移植自 ROSMASTER M3 的 text_chat 功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import time
import os
import signal


def reset_terminal():
    """重置终端状态 / Reset terminal state"""
    os.system("stty sane")


class WheeltecTextChatNode(Node):
    def __init__(self):
        super().__init__('wheeltec_text_chat_node')
        
        # 创建发布者：发送用户输入到 asr 话题
        # Publisher: Send user input to asr topic
        self.asr_publisher = self.create_publisher(String, 'asr', 10)
        
        # 创建订阅者：接收机器人响应
        # Subscriber: Receive robot response
        self.response_subscriber = self.create_subscription(
            String, 'text_response', self.response_callback, 10
        )
        
        # 状态标志 / State flags
        self.running = True
        self.response_received = threading.Event()
        self.latest_response = None
        self.animation_active = False
        self.first_response = True
        
        # 启动动画线程 / Start animation thread
        self.animation_thread = threading.Thread(target=self.display_animation)
        self.animation_thread.daemon = True
        self.animation_thread.start()
        
        # 启动输入线程 / Start input thread
        self.input_thread = threading.Thread(target=self.handle_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("WHEELTEC 文本输入节点已启动")
        self.get_logger().info("WHEELTEC Text Chat Node Started")
        self.get_logger().info("=" * 60)
        self.get_logger().info("提示：请输入您的指令，机器人将执行相应的动作")
        self.get_logger().info("Tip: Enter your commands, robot will execute actions")
        self.get_logger().info("=" * 60)

    def handle_user_input(self):
        """处理用户输入 / Handle user input"""
        while self.running and rclpy.ok():
            try:
                # 读取用户输入 / Read user input
                user_input = input("请输入指令 >>> ").strip()
                
                # UTF-8 编码处理 / UTF-8 encoding handling
                user_input = user_input.encode(
                    'utf-8', errors='replace'
                ).decode('utf-8', errors='replace')
                
                if not user_input:
                    continue
                
                # 发布用户输入到 asr 话题 / Publish user input to asr topic
                self.asr_publisher.publish(String(data=user_input))
                self.get_logger().info(f"已发送指令: {user_input}")
                
                # 设置等待状态 / Set waiting state
                self.first_response = True
                self.response_received.clear()
                self.animation_active = True
                
                # 等待响应（超时10秒）/ Wait for response (10s timeout)
                if self.response_received.wait(timeout=10):
                    self.first_response = False
                else:
                    self.get_logger().warn("响应超时，请检查大模型服务是否正常运行")
                    self.get_logger().warn("Response timeout, please check if AI service is running")
                    self.animation_active = False
                    sys.stdout.write('\r' + ' ' * 60 + '\r')
                    sys.stdout.flush()
                    
            except UnicodeDecodeError as e:
                self.get_logger().error(f"输入编码错误 / Input encoding error: {e}")
                continue
            except EOFError:
                break
            except KeyboardInterrupt:
                self.running = False
                reset_terminal()
                break
            except Exception as e:
                self.get_logger().error(f"输入处理异常 / Input processing error: {e}")
                continue

    def response_callback(self, msg):
        """接收机器人响应的回调函数 / Robot response callback"""
        # 显示响应消息 / Display response message
        self.get_logger().info(f"机器人回复: {msg.data}")
        
        # 首次响应时设置事件 / Set event on first response
        if self.first_response:
            self.response_received.set()
        
        # 停止动画 / Stop animation
        self.animation_active = False
        sys.stdout.write('\r' + ' ' * 60 + '\r')
        sys.stdout.flush()

    def display_animation(self):
        """显示等待动画 / Display waiting animation"""
        animation_chars = ['|', '/', '-', '\\']
        i = 0
        while self.running:
            if self.animation_active:
                sys.stdout.write(
                    f'\r 🤖 机器人正在思考中... {animation_chars[i % len(animation_chars)]}'
                )
                sys.stdout.flush()
                time.sleep(0.1)
                i += 1
            else:
                time.sleep(0.1)


def signal_handler(sig, frame):
    """信号处理器 / Signal handler"""
    reset_terminal()
    sys.exit(0)


def main(args=None):
    """主函数 / Main function"""
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = WheeltecTextChatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.input_thread.join(timeout=1)
        node.animation_thread.join(timeout=1)
        node.destroy_node()
        rclpy.shutdown()
        reset_terminal()


if __name__ == '__main__':
    main()
