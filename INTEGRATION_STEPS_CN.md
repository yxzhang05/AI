# WHEELTEC 语音/文本控制系统集成步骤

## 概述

本文档提供详细的步骤说明，指导如何完整地将 ROSMASTER M3 PRO 的语音和文本控制功能集成到 WHEELTEC 机器人上。

## 前提条件

1. WHEELTEC ROS2 环境已正确安装和配置
2. ROS2 Humble 版本
3. Nav2 导航栈正常工作
4. 已阅读 `MIGRATION_GUIDE_CN.md`

## 集成架构

```
完整系统组成：
├── largemodel_wheeltec     (已创建 - 动作执行层)
├── text_chat              (需要复制 - 文本输入接口)
├── model_service          (需要复制 - 大模型决策层)
├── utils                  (需要复制 - 工具模块)
└── interfaces             (需要复制 - 自定义消息)
```

## 步骤一：复制核心包

### 1.1 复制 interfaces 包

interfaces 包定义了动作消息类型，是系统通信的基础。

```bash
cd ~/wheeltec_ros2/src

# 从 multimoding 复制 interfaces 包
cp -r /path/to/multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/interfaces .

# 或从 Orin 版本复制
# cp -r /path/to/multimoding/Code/Orin/M3Pro_ws/src/interfaces .
```

**验证：**
```bash
ls -la interfaces/
# 应该看到 action 文件夹和 CMakeLists.txt
```

### 1.2 复制 text_chat 包

text_chat 提供文本输入界面。

```bash
cd ~/wheeltec_ros2/src
cp -r /path/to/multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/text_chat .
```

**可选修改：** 如果不需要文本输入功能，可以跳过此步骤。

### 1.3 创建 largemodel 包（完整版本）

由于我们需要 model_service 和相关工具，需要复制完整的 largemodel 包：

```bash
cd ~/wheeltec_ros2/src

# 复制 largemodel 包
cp -r /path/to/multimoding/Code/jetson\ nano_RaspberryPi/M3Pro_ws/src/largemodel .
```

### 1.4 替换 action_service.py

使用我们已经适配的 WHEELTEC 版本：

```bash
cd ~/wheeltec_ros2/src/largemodel/largemodel

# 备份原始文件
mv action_service.py action_service_original.py

# 复制适配版本
cp ~/wheeltec_ros2/src/largemodel_wheeltec/largemodel/action_service_wheeltec.py action_service.py
```

## 步骤二：配置包依赖

### 2.1 修改 largemodel/setup.py

编辑 `~/wheeltec_ros2/src/largemodel/setup.py`，确保 entry_points 正确：

```python
entry_points={
    'console_scripts': [
        'asr = largemodel.asr:main',
        'action_service = largemodel.action_service:main',
        'model_service = largemodel.model_service:main',   
    ],
},
```

### 2.2 更新配置文件

将 WHEELTEC 专用配置复制到 largemodel 包：

```bash
cd ~/wheeltec_ros2/src/largemodel

# 备份原配置
cp config/yahboom.yaml config/yahboom_original.yaml

# 复制 WHEELTEC 配置
cp ~/wheeltec_ros2/src/largemodel_wheeltec/config/wheeltec_config.yaml config/wheeltec_config.yaml

# 复制地图映射配置
cp ~/wheeltec_ros2/src/largemodel_wheeltec/config/map_mapping.yaml config/map_mapping.yaml
```

## 步骤三：配置大模型服务

### 3.1 选择大模型提供商

编辑 `config/wheeltec_config.yaml`，选择大模型服务：

#### 选项 A：使用讯飞星火（适合中国用户）

```yaml
model_service:
  ros__parameters:
    regional_setting: "China"
    # 在 utils/large_model_interface.py 中配置 API 密钥
```

需要在讯飞开放平台申请 API 密钥：https://www.xfyun.cn/

#### 选项 B：使用 OpenAI（适合国际用户）

```yaml
model_service:
  ros__parameters:
    regional_setting: "international"
    # 在 utils/large_model_interface.py 中配置 API 密钥
```

#### 选项 C：使用本地 Ollama（推荐，无需网络）

如果您希望完全离线运行，可以使用 Ollama：

```bash
# 安装 Ollama
curl -fsSL https://ollama.com/install.sh | sh

# 下载模型（例如 qwen）
ollama pull qwen:7b

# 启动 Ollama 服务
ollama serve
```

然后修改 `utils/large_model_interface.py` 以使用本地 Ollama API。

### 3.2 配置 API 密钥

编辑 `~/wheeltec_ros2/src/largemodel/utils/large_model_interface.py`：

```python
# 在文件开头添加您的 API 配置
XUNFEI_API_KEY = "your_api_key_here"
XUNFEI_API_SECRET = "your_api_secret_here"
XUNFEI_APP_ID = "your_app_id_here"

# 或 OpenAI
OPENAI_API_KEY = "your_openai_key_here"
```

**重要：** 不要将 API 密钥提交到 Git 仓库！

## 步骤四：配置语音功能（可选）

### 4.1 语音识别（ASR）配置

如果需要语音控制功能：

#### 检查麦克风硬件

```bash
# 列出音频设备
arecord -l

# 列出串口设备（如果使用串口麦克风）
ls /dev/ttyUSB*
```

#### 配置麦克风参数

编辑 `config/wheeltec_config.yaml`：

```yaml
asr:
  ros__parameters:
    mic_serial_port: "/dev/ttyUSB1"  # 根据实际设备修改
    mic_index: 0                      # 音频设备索引
    use_oline_asr: false              # true=在线识别, false=本地识别
```

### 4.2 语音合成（TTS）配置

```yaml
action_service:
  ros__parameters:
    useolinetts: false  # true=在线合成, false=本地合成
```

对于本地语音合成，需要安装：

```bash
# 安装 pyttsx3（适合离线使用）
pip3 install pyttsx3

# 或安装 espeak
sudo apt-get install espeak
```

### 4.3 语音依赖安装

```bash
pip3 install pyaudio pygame webrtcvad
```

## 步骤五：编译系统

### 5.1 编译所有包

```bash
cd ~/wheeltec_ros2

# 编译 interfaces 包（必须先编译）
colcon build --packages-select interfaces

# 编译其他包
colcon build --packages-select largemodel text_chat

# 加载环境
source install/setup.bash
```

### 5.2 检查编译结果

```bash
# 检查节点是否可用
ros2 pkg executables largemodel

# 应该输出：
# largemodel action_service
# largemodel asr
# largemodel model_service
```

## 步骤六：配置导航目标点

### 6.1 创建地图

首先需要使用 WHEELTEC 的 SLAM 功能创建地图：

```bash
# 启动 SLAM 建图
ros2 launch wheeltec_robot_slam wheeltec_lidar_slam.launch.py

# 使用键盘或手柄控制机器人探索环境
# ...

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 6.2 记录目标点位置

在 rviz2 中记录关键位置的坐标：

1. 在 rviz2 中启用 TF 显示
2. 移动机器人到目标位置
3. 记录 `base_footprint` 相对于 `map` 的坐标

### 6.3 更新 map_mapping.yaml

编辑 `config/map_mapping.yaml`：

```yaml
living_room:
  position:
    x: 1.5    # 从 rviz2 记录的 x 坐标
    y: 2.0    # y 坐标
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707  # 方向（四元数）
    w: 0.707

kitchen:
  position:
    x: 3.0
    y: 1.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## 步骤七：系统测试

### 7.1 基础功能测试

#### 测试 1：启动基础系统

```bash
# 终端 1：启动 WHEELTEC 基础功能
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 2：启动导航
ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py map:=~/maps/my_map.yaml

# 终端 3：启动动作服务
ros2 run largemodel action_service --ros-args --params-file src/largemodel/config/wheeltec_config.yaml
```

#### 测试 2：速度控制测试

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

#### 测试 3：导航测试

在 rviz2 中使用 "2D Goal Pose" 工具设置导航目标。

### 7.2 文本控制测试

```bash
# 终端 1-3：同上启动基础系统

# 终端 4：启动 model service
ros2 run largemodel model_service --ros-args --params-file src/largemodel/config/wheeltec_config.yaml

# 终端 5：启动 text_chat
ros2 run text_chat text_chat
```

然后在 text_chat 终端输入命令，例如：
```
user input: 前进1米
user input: 导航到客厅
user input: 左转90度
```

### 7.3 语音控制测试（如果已配置）

```bash
# 启动 ASR 节点
ros2 run largemodel asr --ros-args --params-file src/largemodel/config/wheeltec_config.yaml
```

唤醒机器人后说出命令。

## 步骤八：创建集成启动文件

为了方便使用，创建一个统一的启动文件。

创建 `~/wheeltec_ros2/src/largemodel/launch/wheeltec_full_system.launch.py`：

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('largemodel')
    params_file = os.path.join(pkg_share, 'config', 'wheeltec_config.yaml')

    # 参数
    text_chat_mode = LaunchConfiguration('text_chat_mode', default='false')
    text_chat_mode_arg = DeclareLaunchArgument(
        'text_chat_mode',
        default_value=text_chat_mode
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

使用方法：

```bash
# 语音模式
ros2 launch largemodel wheeltec_full_system.launch.py

# 文本模式
ros2 launch largemodel wheeltec_full_system.launch.py text_chat_mode:=true
```

## 步骤九：优化和调整

### 9.1 速度参数调整

根据 WHEELTEC 的实际性能，调整速度参数。编辑 `action_service.py`：

```python
def turn_left(self):
    twist = Twist()
    twist.linear.x = 0.3  # 调整线速度
    twist.angular.z = 0.8  # 调整角速度
    # ...
```

### 9.2 大模型提示词优化

编辑 `utils/promot.py`，优化提示词以适配 WHEELTEC 的功能：

```python
def get_prompt():
    prompt = """
    你是一个WHEELTEC机器人控制助手。你可以执行以下动作：
    - navigation(point_name): 导航到目标点
    - move_forward(distance): 前进指定距离（米）
    - turn_left(): 左转弯
    - turn_right(): 右转弯
    - stop(): 停止
    
    注意：WHEELTEC没有机械臂，不能执行抓取动作。
    """
    return prompt
```

### 9.3 添加日志记录

为了调试，建议启用详细日志：

```bash
# 启动时设置日志级别
ros2 run largemodel action_service --ros-args --log-level debug
```

## 步骤十：创建系统服务（可选）

为了开机自启动，可以创建 systemd 服务。

创建 `/etc/systemd/system/wheeltec-voice-control.service`：

```ini
[Unit]
Description=WHEELTEC Voice Control System
After=network.target

[Service]
Type=simple
User=wheeltec
Environment="ROS_DOMAIN_ID=0"
WorkingDirectory=/home/wheeltec/wheeltec_ros2
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch largemodel wheeltec_full_system.launch.py'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

启用服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable wheeltec-voice-control.service
sudo systemctl start wheeltec-voice-control.service
```

## 常见问题解决

### Q1: 编译时找不到 interfaces 包

**A:** 确保先编译 interfaces 包：
```bash
colcon build --packages-select interfaces
source install/setup.bash
colcon build --packages-select largemodel
```

### Q2: 大模型 API 调用失败

**A:** 检查网络连接和 API 密钥配置，或切换到本地模型。

### Q3: 语音识别不准确

**A:** 调整 VAD_MODE 参数或使用在线 ASR 服务。

### Q4: 导航功能不工作

**A:** 
1. 检查地图是否正确加载
2. 确认 Nav2 正常运行
3. 验证 TF 树完整性

### Q5: 机器人响应缓慢

**A:** 
1. 检查计算资源占用
2. 考虑使用更快的大模型
3. 优化提示词长度

## 性能优化建议

1. **使用本地模型**: Ollama + qwen 可以提供良好的性能且无需网络
2. **减少日志输出**: 生产环境中降低日志级别
3. **优化导航参数**: 调整 Nav2 的代价地图更新频率
4. **使用 GPU 加速**: 如果有 GPU，启用深度学习模型加速

## 总结

完成以上步骤后，您的 WHEELTEC 机器人将具备以下能力：

✅ 语音控制移动和导航
✅ 文本输入控制
✅ 自然语言理解（通过大模型）
✅ 自主导航到目标点
✅ 实时状态反馈

建议按顺序完成各步骤，每完成一步都进行测试，确保功能正常后再继续下一步。

## 下一步

- 整合 WHEELTEC 的视觉巡线功能
- 添加多机器人协作控制
- 开发自定义应用场景
- 优化语音交互体验

祝您使用愉快！
