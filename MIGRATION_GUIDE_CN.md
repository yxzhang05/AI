# ROSMASTER M3 PRO 语音/文本控制功能迁移指南

## 概述

本指南详细说明如何将 ROSMASTER M3 PRO 的语音和文本输入控制功能迁移到 WHEELTEC ROS2 机器人平台。

## 系统架构分析

### 源系统 (ROSMASTER M3 PRO)
位于 `multimoding/Code/jetson nano_RaspberryPi/M3Pro_ws/` 和 `multimoding/Code/Orin/M3Pro_ws/`

#### 核心组件

1. **largemodel 包** - 大模型集成包
   - `asr.py`: 自动语音识别节点（ASR）
   - `model_service.py`: 大语言模型服务节点
   - `action_service.py`: 动作执行服务节点
   - `utils/`: 工具模块（语音合成、大模型接口等）

2. **text_chat 包** - 文本交互包
   - `text_chat.py`: 文本输入接口节点

3. **interfaces 包** - 自定义消息接口
   - 动作消息定义

### 目标系统 (WHEELTEC ROS2)
位于 `wheeltec_ros2/src/`

#### 现有功能
- 雷达建图 (SLAM)
- 视觉巡线
- 键盘控制
- 底盘移动控制（通过 `/cmd_vel` 话题）

## ROS2 通信架构

### 话题（Topics）

1. **输入话题**
   - `asr` (std_msgs/String): 语音识别结果或文本输入
   - `wakeup` (std_msgs/Bool): 唤醒信号
   - `text_response` (std_msgs/String): 文本响应

2. **输出话题**
   - `/cmd_vel` (geometry_msgs/Twist): 机器人速度控制
   - `actionstatus` (std_msgs/String): 动作执行状态反馈
   - `tts_topic` (std_msgs/String): 文本转语音

3. **动作服务器**
   - `action_service` (interfaces/action/Rot): 接收动作列表并执行

### 数据流

```
用户输入 (语音/文本)
    ↓
ASR节点 / Text Chat节点 → 发布到 'asr' 话题
    ↓
Model Service节点 → 订阅 'asr' 话题
    ↓
调用大语言模型进行决策
    ↓
生成动作指令列表
    ↓
Action Service节点 → 接收动作请求
    ↓
解析并执行动作（发布 /cmd_vel）
    ↓
反馈执行状态 → 'actionstatus' 话题
```

## 迁移步骤

### 第一步：准备工作

1. 确保 WHEELTEC 机器人的 ROS2 环境正常运行
2. 备份现有配置文件

### 第二步：复制必要的包

需要复制以下包到 `wheeltec_ros2/src/`:

```bash
# 从 multimoding 复制到 wheeltec_ros2
cp -r multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/largemodel wheeltec_ros2/src/
cp -r multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/text_chat wheeltec_ros2/src/
cp -r multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/interfaces wheeltec_ros2/src/
```

### 第三步：修改 action_service.py

ROSMASTER M3 PRO 的 `action_service.py` 包含大量机械臂相关的功能，这些需要移除或禁用，因为 WHEELTEC 是轮式底盘机器人。

**需要保留的功能：**
- 基础移动控制（navigation, set_cmdvel, move_left, move_right, turn_left, turn_right）
- 等待功能（wait）
- 停止功能（stop）
- 获取当前位置（get_current_pose）

**需要移除/禁用的功能：**
- 所有机械臂相关功能（arm_up, arm_down, arm_shake, arm_nod, arm_applaud, dance）
- 抓取相关功能（grasp_obj, putdown, track）
- AprilTag 相关功能（apriltag_sort, apriltag_follow_2D, apriltag_remove_higher）
- 颜色识别相关功能（color_follow_2D, color_remove_higher）
- 巡线功能（follw_line_clear）- 如果 WHEELTEC 已有类似功能

### 第四步：配置参数

创建或修改配置文件 `wheeltec_ros2/src/largemodel/config/wheeltec_config.yaml`:

```yaml
# 控制话题配置
Speed_topic: "/cmd_vel"  # WHEELTEC 的速度控制话题

# 大模型配置
use_double_llm: false
text_chat_mode: false  # true: 文本模式, false: 语音模式
language: "zh"  # 语言设置：zh=中文, en=英文
regional_setting: "China"  # China 或 international

# 语音配置
useolinetts: false  # 是否使用在线TTS
use_oline_asr: false  # 是否使用在线ASR

# 传感器配置（如果有摄像头）
image_topic: "/camera/color/image_raw"

# 麦克风配置
mic_serial_port: "/dev/ttyUSB1"  # 根据实际设备修改
mic_index: 0

# VAD（语音活动检测）配置
VAD_MODE: 1
sample_rate: 48000
frame_duration_ms: 30
```

### 第五步：创建启动文件

创建 `wheeltec_ros2/src/largemodel/launch/wheeltec_voice_control.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    # 配置文件路径
    params_file = os.path.join(
        get_package_share_directory('largemodel'),
        'config',
        'wheeltec_config.yaml'
    )

    # 启动参数
    text_chat_mode = LaunchConfiguration('text_chat_mode', default=False)
    text_chat_mode_arg = DeclareLaunchArgument(
        'text_chat_mode',
        default_value=text_chat_mode,
        description='Enable text chat mode (disable voice)'
    )

    # Model Service 节点
    model_server = Node(
        package='largemodel',
        executable='model_service',
        name='model_service',
        parameters=[
            params_file,
            {'text_chat_mode': text_chat_mode}
        ],
        output='screen'
    )

    # ASR 节点（仅语音模式）
    asr_server = Node(
        package='largemodel',
        executable='asr',
        name='asr',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(text_chat_mode)
    )

    # Action Service 节点
    action_server = Node(
        package='largemodel',
        executable='action_service',
        name='action_service',
        parameters=[
            params_file,
            {'text_chat_mode': text_chat_mode}
        ],
        output='screen'
    )

    return LaunchDescription([
        text_chat_mode_arg,
        model_server,
        action_server,
        asr_server,
    ])
```

### 第六步：编译和测试

```bash
cd ~/wheeltec_ros2
colcon build --packages-select interfaces largemodel text_chat
source install/setup.bash
```

### 第七步：运行系统

**语音控制模式：**
```bash
# 终端1：启动 WHEELTEC 基础功能
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端2：启动语音控制
ros2 launch largemodel wheeltec_voice_control.launch.py
```

**文本控制模式：**
```bash
# 终端1：启动 WHEELTEC 基础功能
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端2：启动文本控制
ros2 launch largemodel wheeltec_voice_control.launch.py text_chat_mode:=True

# 终端3：运行文本输入界面
ros2 run text_chat text_chat
```

## 功能适配说明

### 导航功能集成

WHEELTEC 使用 Nav2 导航栈。需要确保：

1. **地图映射配置**：编辑 `config/map_mapping.yaml` 文件，添加 WHEELTEC 环境中的目标点位置。

```yaml
point1:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

2. **导航功能测试**：
   - 确保 Nav2 正常运行
   - 测试 `navigation(point_name)` 函数

### 速度控制适配

WHEELTEC 的速度控制参数可能需要调整：

```python
# 在 action_service.py 中调整速度参数
def turn_left(self):
    twist = Twist()
    twist.linear.x = 0.3  # 根据 WHEELTEC 调整
    twist.angular.z = 0.8  # 根据 WHEELTEC 调整
    self._execute_action(twist)
```

### 语音硬件配置

如果使用不同的麦克风硬件：

1. 检查麦克风设备：
```bash
ls /dev/tty*  # 查找串口设备
arecord -l     # 查找音频设备
```

2. 修改配置文件中的 `mic_serial_port` 和 `mic_index`

## 依赖项

需要安装以下 Python 包：

```bash
pip3 install pyaudio pygame webrtcvad paramiko psutil opencv-python pyyaml
```

对于在线服务（如果使用）：
- 讯飞语音识别/合成 API
- 大语言模型 API（如 OpenAI、通义千问等）

## 故障排除

### 问题1：找不到麦克风设备
**解决方案：**
```bash
# 检查设备
ls -l /dev/ttyUSB*
# 添加用户权限
sudo usermod -a -G dialout $USER
```

### 问题2：无法连接到大模型服务
**解决方案：**
- 检查网络连接
- 验证 API 密钥配置
- 查看 `utils/large_model_interface.py` 中的配置

### 问题3：机器人不响应控制指令
**解决方案：**
- 检查 `/cmd_vel` 话题是否正确订阅
```bash
ros2 topic echo /cmd_vel
```
- 确认 WHEELTEC 底盘驱动正常运行

### 问题4：语音识别不准确
**解决方案：**
- 调整 VAD_MODE 参数（0-3，越高越严格）
- 检查麦克风音量设置
- 考虑使用在线 ASR 服务以提高准确率

## 功能对照表

| ROSMASTER M3 PRO 功能 | WHEELTEC 适配 | 状态 |
|----------------------|--------------|------|
| 语音唤醒 | ✅ 支持 | 需要麦克风硬件 |
| 文本输入 | ✅ 支持 | 完全兼容 |
| 导航到目标点 | ✅ 支持 | 需配置地图点位 |
| 基础移动控制 | ✅ 支持 | 完全兼容 |
| 转向控制 | ✅ 支持 | 需调整参数 |
| 获取当前位置 | ✅ 支持 | 需 TF 树正常 |
| 机械臂控制 | ❌ 不适用 | WHEELTEC 无机械臂 |
| 物体抓取 | ❌ 不适用 | WHEELTEC 无机械臂 |
| AprilTag 识别 | ⚠️ 可选 | 需摄像头支持 |
| 视觉识别 | ⚠️ 可选 | 需摄像头支持 |

## 扩展建议

1. **整合 WHEELTEC 现有功能**：
   - 将视觉巡线功能添加到动作列表
   - 整合雷达建图功能

2. **添加自定义动作**：
   - 在 `action_service.py` 中添加 WHEELTEC 特定的动作函数

3. **多机器人协作**：
   - 利用 WHEELTEC 的多机协作功能
   - 扩展语音控制支持多机器人指令

## 总结

本迁移方案保留了 ROSMASTER M3 PRO 的核心语音/文本输入和大模型决策功能，同时移除了 WHEELTEC 不需要的机械臂控制功能。通过适当的参数配置和功能适配，可以实现在 WHEELTEC 机器人上使用语音和文本指令控制机器人移动、导航等功能。

关键要点：
- 使用 WHEELTEC 原有的底盘控制（通过 `/cmd_vel` 话题）
- 移除机械臂相关代码
- 配置正确的硬件接口（麦克风、摄像头等）
- 根据实际机器人性能调整速度参数

## 参考资料

- ROS2 Humble 文档：https://docs.ros.org/en/humble/
- Nav2 导航文档：https://navigation.ros.org/
- WHEELTEC 官方文档（如有）
- 大语言模型 API 文档

## 联系与支持

如有问题，请参考：
- 项目 Issue 页面
- ROS2 社区论坛
- WHEELTEC 技术支持
