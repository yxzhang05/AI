#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC Robot Action Service - Adapted from ROSMASTER M3 PRO
This version removes arm-related functions and focuses on mobile base control
"""

import re
import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class WheeltecActionServer(Node):
    def __init__(self):
        super().__init__("wheeltec_action_service")
        
        # 初始化参数配置 / Initialize parameter configuration
        self.init_param_config()
        # 初始化ROS通信 / Initialize ROS communication
        self.init_ros_comunication()
        # 加载地图映射文件 / Load map mapping file
        self.load_target_points()
        # 初始化语言设置 / Initialize language settings
        self.init_language()
        
        self.get_logger().info("WHEELTEC action service started...")

    def init_param_config(self):
        """初始化参数配置 / Initialize parameter configuration"""
        # 设置配置文件路径
        pkg_share = get_package_share_directory("largemodel_wheeltec")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        
        # 声明参数
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("language", "zh")
        
        # 获取参数值
        self.Speed_topic = self.get_parameter("Speed_topic").get_parameter_value().string_value
        self.text_chat_mode = self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        self.language = self.get_parameter("language").get_parameter_value().string_value
        
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        self.combination_mode = False  # 组合模式
        self.interrupt_flag = False  # 打断标志
        self.action_runing = False  # 动作执行状态
        self.first_record = True  # 首次记录位置

    def init_ros_comunication(self):
        """初始化创建ROS通信对象、函数 / Initialize ROS communication"""
        # 创建速度话题发布者
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        
        # 创建导航功能客户端
        self.navclient = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # 创建动作执行服务器（暂时注释，等待定义接口包）
        # self._action_server = ActionServer(
        #     self, Rot, "action_service", self.execute_callback
        # )
        
        # 创建执行动作状态发布者
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        
        # 创建tf监听者
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建打断状态发布者
        self.interrupt_flag_pub = self.create_publisher(Bool, "interrupt_flag", 1)
        
        # wakeup话题订阅者
        self.wakeup_sub = self.create_subscription(
            Bool, "wakeup", self.wakeup_callback, 5
        )

    def init_language(self):
        """初始化语言设置 / Initialize language settings"""
        language_list = ["zh", "en"]
        if self.language not in language_list:
            self.get_logger().error(
                "语言设置错误，请检查配置文件中的 language 参数"
            )
            self.language = "zh"

        self.feedback_dict = {
            "zh": {
                "navigation_1": "机器人反馈:导航目标{point_name}被拒绝",
                "navigation_2": "机器人反馈:执行navigation({point_name})完成",
                "navigation_3": "机器人反馈:执行navigation({point_name})失败，目标点不存在",
                "navigation_4": "机器人反馈:执行navigation({point_name})失败",
                "get_current_pose_success": "机器人反馈:get_current_pose()成功",
                "wait_done": "机器人反馈:执行wait({duration})完成",
                "set_cmdvel_done": "机器人反馈:执行set_cmdvel({linear_x},{linear_y},{angular_z},{duration})完成",
                "move_left_done": "机器人反馈:执行move_left({angle},{angular_speed})完成",
                "move_right_done": "机器人反馈:执行move_right({angle},{angular_speed})完成",
                "turn_left_done": "机器人反馈:执行turn_left()完成",
                "turn_right_done": "机器人反馈:执行turn_right()完成",
                "move_forward_done": "机器人反馈:执行move_forward({distance})完成",
                "move_backward_done": "机器人反馈:执行move_backward({distance})完成",
                "response_done": "机器人反馈：回复用户完成",
                "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
                "finish": "finish",
                "multiple_done": "机器人反馈：执行{actions}完成",
            },
            "en": {
                "navigation_1": "Robot feedback: Navigation target {point_name} rejected",
                "navigation_2": "Robot feedback: Execute navigation({point_name}) completed",
                "navigation_3": "Robot feedback: Execute navigation({point_name}) failed, target does not exist",
                "navigation_4": "Robot feedback: Execute navigation({point_name}) failed",
                "get_current_pose_success": "Robot feedback: get_current_pose() succeeded",
                "wait_done": "Robot feedback: Execute wait({duration}) completed",
                "set_cmdvel_done": "Robot feedback: Execute set_cmdvel({linear_x},{linear_y},{angular_z},{duration}) completed",
                "move_left_done": "Robot feedback: Execute move_left({angle},{angular_speed}) completed",
                "move_right_done": "Robot feedback: Execute move_right({angle},{angular_speed}) completed",
                "turn_left_done": "Robot feedback: Execute turn_left() completed",
                "turn_right_done": "Robot feedback: Execute turn_right() completed",
                "move_forward_done": "Robot feedback: Execute move_forward({distance}) completed",
                "move_backward_done": "Robot feedback: Execute move_backward({distance}) completed",
                "response_done": "Robot feedback: Reply to user completed",
                "failure_execute_action_function_not_exists": "Robot feedback: Action function not exists",
                "finish": "finish",
                "multiple_done": "Robot feedback: Execution {actions} completed",
            },
        }

    def load_target_points(self):
        """加载地图映射文件 / Load map mapping file"""
        if os.path.exists(self.map_mapping_config):
            with open(self.map_mapping_config, "r") as file:
                target_points = yaml.safe_load(file)
            self.navpose_dict = {}
            if target_points:
                for name, data in target_points.items():
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = data["position"]["x"]
                    pose.pose.position.y = data["position"]["y"]
                    pose.pose.position.z = data["position"]["z"]
                    pose.pose.orientation.x = data["orientation"]["x"]
                    pose.pose.orientation.y = data["orientation"]["y"]
                    pose.pose.orientation.z = data["orientation"]["z"]
                    pose.pose.orientation.w = data["orientation"]["w"]
                    self.navpose_dict[name] = pose
        else:
            self.navpose_dict = {}
            self.get_logger().warn(f"地图映射文件不存在: {self.map_mapping_config}")

    def wakeup_callback(self, msg):
        """唤醒打断回调函数 / Wake-up interrupt callback function"""
        if msg.data:
            if self.action_runing:
                self.interrupt_flag = True
                self.stop()

    def action_status_pub(self, key, **kwargs):
        """发布动作状态消息 / Publish action status message"""
        text_template = self.feedback_dict[self.language].get(key)
        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"
        
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")

    def get_current_pose(self):
        """获取当前在全局地图坐标系下的位置 / Get current position in global map"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation
            self.navpose_dict["zero"] = pose
            
            position = pose.pose.position
            orientation = pose.pose.orientation
            self.get_logger().info(
                f"Recorded Pose - Position: x={position.x}, y={position.y}, z={position.z}"
            )
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_success")
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")

    def navigation(self, point_name):
        """导航到目标点 / Navigate to target point"""
        self.navigation_finish_flag = False
        self.goal_handle = None
        self.result = None
        point_name = point_name.strip("'\"")
        
        if point_name not in self.navpose_dict:
            self.get_logger().error(
                f"Target point '{point_name}' does not exist in navigation dictionary."
            )
            self.action_status_pub("navigation_3", point_name=point_name)
            return

        if self.first_record:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time()
                )
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = 0.0
                pose.pose.orientation = transform.transform.rotation
                self.navpose_dict["zero"] = pose
                self.first_record = False
            except Exception as e:
                self.get_logger().error(f"Failed to record starting pose: {e}")

        target_pose = self.navpose_dict.get(point_name)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        send_goal_future = self.navclient.send_goal_async(goal_msg)

        def goal_response_callback(future):
            self.goal_handle = future.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                self.navigation_finish_flag = True
                return

            get_result_future = self.goal_handle.get_result_async()

            def result_callback(future_result):
                self.result = future_result.result()
                self.navigation_finish_flag = True
                if self.result.status == 4:
                    self.action_status_pub("navigation_2", point_name=point_name)
                elif self.result.status == 5:
                    self.get_logger().info("Navigation cancelled")
                else:
                    self.get_logger().info(
                        f"Navigation failed with status: {self.result.status}"
                    )
                    self.action_status_pub("navigation_4", point_name=point_name)

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.navclient._cancel_goal(self.goal_handle)
                break
            time.sleep(0.1)
        self.stop()

    def wait(self, duration):
        """等待指定时间 / Wait for specified duration"""
        duration = float(duration)
        time.sleep(duration)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)

    def set_cmdvel(self, linear_x, linear_y, angular_z, duration):
        """发布速度命令 / Publish velocity command"""
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._execute_action(twist, durationtime=duration)
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "set_cmdvel_done",
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z,
                duration=duration,
            )

    def move_left(self, angle, angular_speed):
        """左转指定角度 / Turn left by specified angle"""
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)
        duration = abs(angle_rad / angular_speed)
        angular_speed = abs(angular_speed)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_left_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def move_right(self, angle, angular_speed):
        """右转指定角度 / Turn right by specified angle"""
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)
        duration = abs(angle_rad / angular_speed)
        angular_speed = -abs(angular_speed)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_right_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def turn_left(self):
        """左转弯 / Turn left"""
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.8
        self._execute_action(twist)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("turn_left_done")

    def turn_right(self):
        """右转弯 / Turn right"""
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = -0.8
        self._execute_action(twist)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("turn_right_done")

    def move_forward(self, distance):
        """前进指定距离 / Move forward by specified distance"""
        distance = float(distance)
        speed = 0.3
        duration = abs(distance / speed)
        
        twist = Twist()
        twist.linear.x = speed if distance > 0 else -speed
        twist.angular.z = 0.0
        self._execute_action(twist, durationtime=duration)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("move_forward_done", distance=distance)

    def move_backward(self, distance):
        """后退指定距离 / Move backward by specified distance"""
        distance = float(distance)
        speed = 0.3
        duration = abs(distance / speed)
        
        twist = Twist()
        twist.linear.x = -speed
        twist.angular.z = 0.0
        self._execute_action(twist, durationtime=duration)
        self.stop()
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("move_backward_done", distance=distance)

    def stop(self):
        """停止机器人 / Stop robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def _execute_action(self, twist, num=1, durationtime=3.0):
        """执行动作 / Execute action"""
        for _ in range(num):
            start_time = time.time()
            while (time.time() - start_time) < durationtime:
                if self.interrupt_flag:
                    self.stop()
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)

    def finish_dialogue(self):
        """结束对话 / Finish dialogue"""
        self.first_record = True
        self.action_status_pub("finish")

    def finishtask(self):
        """空操作 / Empty operation"""
        return


def main(args=None):
    rclpy.init(args=args)
    wheeltec_action_server = WheeltecActionServer()
    
    try:
        rclpy.spin(wheeltec_action_server)
    except KeyboardInterrupt:
        wheeltec_action_server.stop()
    finally:
        wheeltec_action_server.stop()
        wheeltec_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
