# WHEELTEC 文本输入功能移植 - 实施文件

## 文件说明

此文件夹包含将 ROSMASTER M3 的文本输入功能移植到 WHEELTEC 机器人所需的所有文件。

### 文件列表

1. **largemodel/text_chat_wheeltec.py**
   - 文本输入节点的主要实现
   - 基于 ROSMASTER M3 的 text_chat.py 改编
   - 提供命令行文本输入界面

2. **launch/wheeltec_text_control.launch.py**
   - 启动文件，同时启动动作服务和文本输入节点
   - 用于文本控制模式

3. **此 README 文件**
   - 实施指南

## 安装步骤

### 第一步：复制文件到 WHEELTEC 包

```bash
# 假设您在 AI 仓库根目录

# 1. 复制 text_chat_wheeltec.py 到 largemodel 模块
cp wheeltec_text_input_port/largemodel/text_chat_wheeltec.py \
   wheeltec_ros2/src/largemodel_wheeltec/largemodel/

# 2. 复制启动文件
cp wheeltec_text_input_port/launch/wheeltec_text_control.launch.py \
   wheeltec_ros2/src/largemodel_wheeltec/launch/
```

### 第二步：修改 setup.py

编辑 `wheeltec_ros2/src/largemodel_wheeltec/setup.py`，在 `entry_points` 部分添加 text_chat 入口点：

```python
entry_points={
    'console_scripts': [
        'action_service = largemodel.action_service_wheeltec:main',
        'text_chat = largemodel.text_chat_wheeltec:main',  # 新增这一行
    ],
},
```

完整的 `entry_points` 部分应该如下：

```python
    entry_points={
        'console_scripts': [
            'action_service = largemodel.action_service_wheeltec:main',
            'text_chat = largemodel.text_chat_wheeltec:main',
        ],
    },
```

### 第三步：编译包

```bash
cd ~/wheeltec_ros2

# 编译 largemodel_wheeltec 包
colcon build --packages-select largemodel_wheeltec

# 加载环境变量
source install/setup.bash
```

### 第四步：验证安装

```bash
# 检查 text_chat 是否可用
ros2 pkg executables largemodel_wheeltec

# 应该看到输出：
# action_service
# text_chat
```

## 使用方法

### 方法一：使用启动文件（推荐）

```bash
# 终端 1: 启动 WHEELTEC 机器人基础功能
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 2: 启动大模型服务（需要按照 course 文件夹教程配置）
# [用户需要自行配置]

# 终端 3: 启动文本控制（包含 action_service 和 text_chat）
ros2 launch largemodel_wheeltec wheeltec_text_control.launch.py
```

### 方法二：单独运行文本输入节点

```bash
# 如果 action_service 已经在运行，可以单独启动 text_chat
ros2 run largemodel_wheeltec text_chat
```

### 测试文本输入节点（不含大模型）

如果您想测试文本输入节点是否正常工作，而不需要配置大模型：

```bash
# 终端 1: 启动文本输入节点
ros2 run largemodel_wheeltec text_chat

# 终端 2: 监听 /asr 话题（查看用户输入）
ros2 topic echo /asr

# 终端 3: 手动发送响应（模拟大模型服务）
ros2 topic pub /text_response std_msgs/String "data: '这是测试响应消息'" --once
```

## 使用示例

启动文本控制后，您可以这样与机器人交互：

```
请输入指令 >>> 前进1米
🤖 机器人正在思考中... |
机器人回复: 机器人反馈:执行move_forward(1.0)完成

请输入指令 >>> 左转90度
🤖 机器人正在思考中... /
机器人回复: 机器人反馈:执行move_left(90,0.5)完成

请输入指令 >>> 停止
🤖 机器人正在思考中... -
机器人回复: 机器人反馈:执行stop()完成
```

## 常见问题

### Q1: 节点启动失败，提示找不到 text_chat

**A:** 检查是否正确编译并加载了环境变量：

```bash
cd ~/wheeltec_ros2
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

### Q2: 输入指令后一直显示"思考中"，最后超时

**A:** 这说明文本输入节点工作正常，但是没有收到来自大模型服务的响应。请检查：

1. 大模型服务是否已启动
2. 大模型服务是否正确订阅了 `/asr` 话题
3. 大模型服务是否能正确发布到 `/text_response` 话题

可以手动测试：
```bash
ros2 topic pub /text_response std_msgs/String "data: '测试'" --once
```

### Q3: 中文显示乱码

**A:** 设置终端编码为 UTF-8：

```bash
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
```

或使用支持 UTF-8 的终端（如 GNOME Terminal）。

### Q4: 如何修改响应超时时间？

**A:** 编辑 `text_chat_wheeltec.py`，找到这一行：

```python
if self.response_received.wait(timeout=10):
```

将 `10` 改为您需要的秒数，例如 `30`。

## 架构说明

文本输入系统的完整架构：

```
用户输入
   ↓
[text_chat 节点]
   ↓ 发布到 /asr
   ↓
[大模型服务] ← 用户需要自行配置（参考 course 教程）
   ↓ 解析意图，调用函数
   ↓
[action_service 节点]
   ↓ 执行动作
   ↓ 发布到 /cmd_vel
   ↓
[WHEELTEC 底盘驱动]
   ↓
[机器人执行动作]
   ↓ 状态反馈 /actionstatus
   ↓
[大模型服务]
   ↓ 生成回复 /text_response
   ↓
[text_chat 节点显示]
```

## 话题接口

文本输入节点使用以下 ROS2 话题：

| 话题名 | 消息类型 | 方向 | 说明 |
|--------|----------|------|------|
| `/asr` | std_msgs/String | 发布 | 用户输入的文本指令 |
| `/text_response` | std_msgs/String | 订阅 | 系统响应消息 |

## 下一步

1. ✅ 完成文本输入节点移植
2. ⏭️ 配置大模型服务（参考 `course` 文件夹教程）
3. ⏭️ 测试完整的文本控制流程
4. ⏭️ 根据需要定制机器人动作函数
5. ⏭️ 可选：添加语音输入支持

## 参考文档

- **详细移植分析**: 参见仓库根目录的 `文本输入功能移植分析.md`
- **WHEELTEC 包文档**: `wheeltec_ros2/src/largemodel_wheeltec/README_CN.md`
- **大模型配置教程**: `course/` 文件夹中的 PDF 文件

## 许可证

Apache-2.0（与 WHEELTEC 原包保持一致）

---

**版本**: 1.0  
**最后更新**: 2024-11-19  
**适用系统**: WHEELTEC ROS2 Humble on Jetson Orin NX
