# WHEELTEC 机器人语音/文本控制包

## 简介

本包为 WHEELTEC ROS2 机器人提供语音和文本输入控制功能，基于 ROSMASTER M3 PRO 的大模型控制系统改编。

## 功能特点

- ✅ **移动控制**: 前进、后退、左转、右转
- ✅ **导航功能**: 导航到预定义的目标点
- ✅ **速度控制**: 自定义线速度和角速度
- ✅ **位置获取**: 获取机器人当前位置
- ✅ **中断支持**: 支持语音唤醒打断当前动作
- ✅ **多语言支持**: 支持中文和英文

## 与 ROSMASTER M3 PRO 的区别

本包是简化版本，专注于移动底盘控制：

| 功能 | ROSMASTER M3 PRO | WHEELTEC |
|------|------------------|----------|
| 移动底盘控制 | ✅ | ✅ |
| 导航功能 | ✅ | ✅ |
| 机械臂控制 | ✅ | ❌ 已移除 |
| 物体抓取 | ✅ | ❌ 已移除 |
| AprilTag 识别 | ✅ | ❌ 已移除 |
| 视觉识别 | ✅ | ❌ 已移除 |

## 依赖项

### ROS2 包依赖
- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `nav2_msgs`
- `tf2_ros`
- `ament_index_python`

### Python 依赖
```bash
pip3 install pyyaml
```

## 安装

1. 将本包复制到 WHEELTEC 工作空间：
```bash
cd ~/wheeltec_ros2/src
# 本包已存在于 src/largemodel_wheeltec
```

2. 编译包：
```bash
cd ~/wheeltec_ros2
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

## 配置

### 1. 编辑配置文件

配置文件位于 `config/wheeltec_config.yaml`，主要参数：

```yaml
action_service:
  ros__parameters:
    Speed_topic: "/cmd_vel"      # 速度控制话题
    text_chat_mode: false        # 是否启用文本模式
    language: "zh"               # 语言设置: zh/en
```

### 2. 配置导航目标点

编辑 `config/map_mapping.yaml` 添加您的目标点：

```yaml
living_room:
  position:
    x: 1.5
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## 使用方法

### 基础启动

1. 启动 WHEELTEC 机器人基础功能：
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

2. 启动动作服务：
```bash
ros2 launch largemodel_wheeltec wheeltec_voice_control.launch.py
```

### 测试动作执行

使用命令行测试动作：

```bash
# 测试前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 测试左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

## 支持的动作函数

以下是可以通过大模型调用的动作函数：

### 移动控制
- `move_forward(distance)` - 前进指定距离（米）
- `move_backward(distance)` - 后退指定距离（米）
- `turn_left()` - 左转弯
- `turn_right()` - 右转弯
- `move_left(angle, angular_speed)` - 原地左转指定角度
- `move_right(angle, angular_speed)` - 原地右转指定角度
- `stop()` - 停止

### 导航功能
- `navigation(point_name)` - 导航到指定目标点
- `get_current_pose()` - 获取当前位置

### 速度控制
- `set_cmdvel(linear_x, linear_y, angular_z, duration)` - 设置速度并持续指定时间

### 辅助功能
- `wait(duration)` - 等待指定秒数
- `finish_dialogue()` - 结束当前对话

## 话题接口

### 订阅的话题
- `/wakeup` (std_msgs/Bool) - 唤醒信号，用于打断当前动作

### 发布的话题
- `/cmd_vel` (geometry_msgs/Twist) - 速度控制命令
- `/actionstatus` (std_msgs/String) - 动作执行状态反馈
- `/interrupt_flag` (std_msgs/Bool) - 打断标志

## 故障排除

### 问题1: 机器人不响应控制命令

**检查步骤：**
1. 确认 `/cmd_vel` 话题是否正常：
   ```bash
   ros2 topic echo /cmd_vel
   ```
2. 确认 WHEELTEC 底盘驱动正常运行
3. 检查配置文件中的 `Speed_topic` 参数

### 问题2: 导航功能不工作

**检查步骤：**
1. 确认 Nav2 导航栈正常运行：
   ```bash
   ros2 node list | grep nav
   ```
2. 检查 TF 树是否正常：
   ```bash
   ros2 run tf2_tools view_frames
   ```
3. 确认 `map_mapping.yaml` 中定义了目标点

### 问题3: 编译错误

**解决方案：**
```bash
# 清理构建文件
cd ~/wheeltec_ros2
rm -rf build install log
# 重新编译
colcon build --packages-select largemodel_wheeltec
```

## 扩展开发

### 添加自定义动作

1. 在 `action_service_wheeltec.py` 中添加新方法：

```python
def my_custom_action(self, param1, param2):
    """自定义动作描述"""
    # 实现动作逻辑
    # ...
    
    if not self.combination_mode and not self.interrupt_flag:
        self.action_status_pub("custom_action_done")
```

2. 在 `init_language()` 中添加反馈消息：

```python
"custom_action_done": "机器人反馈:执行my_custom_action完成",
```

### 整合 WHEELTEC 现有功能

可以将 WHEELTEC 的现有功能（如视觉巡线）集成到动作系统中：

```python
def start_line_following(self):
    """启动视觉巡线"""
    # 调用 WHEELTEC 的巡线功能
    # ...
```

## 注意事项

1. **安全距离**: 在室内测试时确保周围有足够空间
2. **速度限制**: 建议初始测试时使用较低速度
3. **紧急停止**: 可以通过发布 `/wakeup` 话题打断当前动作
4. **导航精度**: 导航精度取决于 WHEELTEC 的定位系统质量

## 版本历史

- **v1.0.0** (2024-11-04)
  - 初始版本
  - 从 ROSMASTER M3 PRO 移植基础功能
  - 移除机械臂和视觉识别功能
  - 适配 WHEELTEC 底盘控制

## 许可证

Apache-2.0

## 参考

- WHEELTEC ROS2 文档
- Nav2 导航文档: https://navigation.ros.org/
- ROS2 Humble 文档: https://docs.ros.org/en/humble/

## 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。
