# WHEELTEC 文本输入功能快速开始指南

## 📋 概述

本指南帮助您快速将 ROSMASTER M3 的文本输入功能移植到 WHEELTEC 机器人。

## 🎯 移植目标

- ✅ 添加文本输入界面
- ✅ 复用 WHEELTEC 现有的动作控制系统
- ✅ 保持与大模型服务的兼容性

## 📦 准备工作

### 系统要求
- WHEELTEC 机器人（Jetson Orin NX）
- ROS2 Humble
- Python 3.8+
- 已安装的 WHEELTEC 基础软件包

### 确认现有功能
```bash
# 检查 wheeltec_ros2 是否存在
ls ~/wheeltec_ros2/src/largemodel_wheeltec

# 检查包是否已编译
ros2 pkg list | grep largemodel_wheeltec
```

## 🚀 快速安装（3个步骤）

### 步骤 1: 获取代码

```bash
# 如果您在 GitHub 仓库中
cd /path/to/AI  # 您的仓库路径

# 复制文本输入节点代码
cp wheeltec_text_input_port/largemodel/text_chat_wheeltec.py \
   ~/wheeltec_ros2/src/largemodel_wheeltec/largemodel/

# 复制启动文件
cp wheeltec_text_input_port/launch/wheeltec_text_control.launch.py \
   ~/wheeltec_ros2/src/largemodel_wheeltec/launch/
```

### 步骤 2: 更新 setup.py

```bash
# 编辑 setup.py
nano ~/wheeltec_ros2/src/largemodel_wheeltec/setup.py
```

找到 `entry_points` 部分，添加一行：

```python
entry_points={
    'console_scripts': [
        'action_service = largemodel.action_service_wheeltec:main',
        'text_chat = largemodel.text_chat_wheeltec:main',  # 添加这行
    ],
},
```

保存并退出（Ctrl+O, Enter, Ctrl+X）

### 步骤 3: 编译

```bash
cd ~/wheeltec_ros2
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

## ✅ 验证安装

```bash
# 检查是否安装成功
ros2 pkg executables largemodel_wheeltec

# 应该看到：
# action_service
# text_chat  ← 这是新增的
```

## 🎮 使用方法

### 完整启动流程

打开 **3 个终端**：

**终端 1 - 启动机器人底盘**：
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

**终端 2 - 启动大模型服务**：
```bash
# 请按照 course 文件夹中的教程配置您的大模型服务
# 教程文件：
#   - 1.Semantic understand and command follow.pdf
#   - 5.Configure AI large model.pdf
```

**终端 3 - 启动文本控制**：
```bash
ros2 launch largemodel_wheeltec wheeltec_text_control.launch.py
```

### 简单测试（不需要大模型）

如果您只想测试文本输入节点本身：

**终端 1**：
```bash
ros2 run largemodel_wheeltec text_chat
```

**终端 2**：
```bash
# 监听用户输入
ros2 topic echo /asr
```

**终端 3**：
```bash
# 手动发送响应
ros2 topic pub /text_response std_msgs/String "data: '测试成功！'" --once
```

## 💬 使用示例

启动后，您将看到：

```
[INFO] [wheeltec_text_chat_node]: ============================================================
[INFO] [wheeltec_text_chat_node]: WHEELTEC 文本输入节点已启动
[INFO] [wheeltec_text_chat_node]: WHEELTEC Text Chat Node Started
[INFO] [wheeltec_text_chat_node]: ============================================================
[INFO] [wheeltec_text_chat_node]: 提示：请输入您的指令，机器人将执行相应的动作
[INFO] [wheeltec_text_chat_node]: Tip: Enter your commands, robot will execute actions
[INFO] [wheeltec_text_chat_node]: ============================================================
请输入指令 >>> 
```

**示例对话**：

```
请输入指令 >>> 前进1米
[INFO] [wheeltec_text_chat_node]: 已发送指令: 前进1米
🤖 机器人正在思考中... |
[INFO] [wheeltec_text_chat_node]: 机器人回复: 机器人反馈:执行move_forward(1.0)完成

请输入指令 >>> 左转90度
[INFO] [wheeltec_text_chat_node]: 已发送指令: 左转90度
🤖 机器人正在思考中... /
[INFO] [wheeltec_text_chat_node]: 机器人回复: 机器人反馈:执行move_left(90,0.5)完成

请输入指令 >>> 停止
[INFO] [wheeltec_text_chat_node]: 已发送指令: 停止
[INFO] [wheeltec_text_chat_node]: 机器人回复: 机器人反馈:执行stop()完成
```

## 🐛 故障排除

### 问题：找不到 text_chat 命令

**解决方案**：
```bash
# 重新编译
cd ~/wheeltec_ros2
rm -rf build install log
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

### 问题：输入后一直"思考中"，最后超时

**原因**：大模型服务未启动或未正确配置

**解决方案**：
1. 确认大模型服务是否运行：
   ```bash
   ros2 topic list | grep -E "(asr|text_response)"
   ```

2. 手动测试响应：
   ```bash
   ros2 topic pub /text_response std_msgs/String "data: '测试'" --once
   ```

### 问题：中文显示乱码

**解决方案**：
```bash
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
```

## 📚 支持的指令示例

以下指令需要大模型服务支持：

### 基础移动
- "前进2米"
- "后退1米"  
- "左转90度"
- "右转45度"
- "停止"

### 导航（需要先配置目标点）
- "去客厅"
- "导航到point1"
- "回到home"

### 查询
- "当前位置在哪"
- "获取当前位置"

### 组合指令
- "前进1米然后左转"
- "去point1然后停止"

## 📖 完整文档

- **详细分析文档**: `文本输入功能移植分析.md`
- **实施指南**: `wheeltec_text_input_port/README.md`
- **大模型配置**: `course/` 文件夹

## 🔧 自定义配置

### 修改超时时间

编辑 `text_chat_wheeltec.py`：
```python
if self.response_received.wait(timeout=10):  # 改为您需要的秒数
```

### 修改提示信息

编辑 `text_chat_wheeltec.py` 中的提示文本以适应您的需求。

## 🎉 下一步

1. ✅ 完成基础安装
2. ⬜ 配置大模型服务（参考 course 教程）
3. ⬜ 测试所有动作函数
4. ⬜ 配置导航目标点
5. ⬜ 可选：添加语音输入

## 📞 需要帮助？

查看以下资源：
- 仓库 Issues
- WHEELTEC 官方文档
- ROS2 Humble 文档：https://docs.ros.org/en/humble/

---

**祝您使用愉快！** 🚀

如有问题，请参考详细文档或提交 Issue。
