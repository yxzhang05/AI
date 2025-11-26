# AI - WHEELTEC 机器人文本输入功能移植项目

## 📌 项目简介

本项目将 ROSMASTER M3 机器人的文本输入控制功能移植到 WHEELTEC 机器人（Jetson Orin NX + ROS2 Humble），使用户能够通过命令行文本输入控制机器人的移动和导航功能。

## 🚀 快速开始

### 方式一：查看快速开始指南
阅读 **[QUICKSTART_CN.md](./QUICKSTART_CN.md)** - 3步快速安装和使用

### 方式二：查看完整文档
1. **[文本输入功能移植分析.md](./文本输入功能移植分析.md)** - 详细技术分析（16KB，推荐开发者阅读）
2. **[系统架构图解.md](./系统架构图解.md)** - 可视化架构说明
3. **[PROJECT_SUMMARY_CN.md](./PROJECT_SUMMARY_CN.md)** - 项目总结

## 📁 项目结构

```
AI/
├── README.md                        # 本文件
├── QUICKSTART_CN.md                 # 快速开始指南 ⭐
├── 文本输入功能移植分析.md          # 详细技术文档 ⭐
├── 系统架构图解.md                  # 架构可视化
├── PROJECT_SUMMARY_CN.md            # 项目总结
├── wheeltec_text_input_port/        # 实施文件包 ⭐
│   ├── largemodel/
│   │   └── text_chat_wheeltec.py    # 文本输入节点
│   ├── launch/
│   │   └── wheeltec_text_control.launch.py
│   └── README.md                    # 实施指南
└── course/                          # 大模型配置教程
    ├── 1.Semantic understand and command follow.pdf
    └── 5.Configure AI large model.pdf
```

## ✨ 核心功能

- ✅ **命令行文本输入** - 通过终端输入控制机器人
- ✅ **实时反馈** - 等待动画和状态提示
- ✅ **中文支持** - 完整的中文界面和文档
- ✅ **ROS2 集成** - 标准话题通信接口
- ✅ **易于安装** - 3步快速部署

## 🎯 支持的功能

### 基础移动
- 前进/后退指定距离
- 左转/右转
- 原地旋转指定角度
- 停止

### 导航功能
- 导航到预定义目标点
- 获取当前位置

### 速度控制
- 自定义线速度和角速度

## 📖 文档导航

### 新手用户
👉 从这里开始：**[QUICKSTART_CN.md](./QUICKSTART_CN.md)**

### 开发者
👉 技术深入：**[文本输入功能移植分析.md](./文本输入功能移植分析.md)**

### 系统架构
👉 架构理解：**[系统架构图解.md](./系统架构图解.md)**

## 🛠️ 安装步骤（简要）

```bash
# 1. 复制文件
cp wheeltec_text_input_port/largemodel/text_chat_wheeltec.py \
   ~/wheeltec_ros2/src/largemodel_wheeltec/largemodel/

cp wheeltec_text_input_port/launch/wheeltec_text_control.launch.py \
   ~/wheeltec_ros2/src/largemodel_wheeltec/launch/

# 2. 更新 setup.py
# 在 entry_points 中添加:
#   'text_chat = largemodel.text_chat_wheeltec:main',

# 3. 编译
cd ~/wheeltec_ros2
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

详细步骤请查看 [QUICKSTART_CN.md](./QUICKSTART_CN.md)

## 🎮 使用示例

```bash
# 启动文本控制
ros2 launch largemodel_wheeltec wheeltec_text_control.launch.py
```

```
请输入指令 >>> 前进1米
🤖 机器人正在思考中... |
机器人回复: 机器人反馈:执行move_forward(1.0)完成

请输入指令 >>> 左转90度
🤖 机器人正在思考中... /
机器人回复: 机器人反馈:执行move_left(90,0.5)完成
```

## ⚠️ 重要提示

**大模型服务需要用户自行配置**

本项目仅提供文本输入/输出接口，大模型服务需要按照 `course` 文件夹中的教程进行配置。

## 🔧 系统要求

- WHEELTEC 机器人（Jetson Orin NX）
- ROS2 Humble
- Python 3.8+
- 已安装的 WHEELTEC 基础软件包

## 📚 参考文档

- [WHEELTEC 官方文档](https://wheeltec.net/)（如有）
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 导航文档](https://navigation.ros.org/)

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

Apache-2.0（与 WHEELTEC 原包保持一致）

## 🎉 致谢

- WHEELTEC ROS2 机器人软件包
- ROSMASTER M3 文本输入功能
- ROS2 社区

---

**版本**: 1.0.0  
**更新日期**: 2024-11-19  
**适用平台**: WHEELTEC on Jetson Orin NX with ROS2 Humble