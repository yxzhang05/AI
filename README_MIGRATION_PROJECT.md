# ROSMASTER M3 PRO 到 WHEELTEC 语音/文本控制功能迁移项目

## 项目概述

本项目实现了将 ROSMASTER M3 PRO 机器人的语音和文本输入控制功能迁移到 WHEELTEC ROS2 机器人平台。通过适配和优化，WHEELTEC 机器人现在可以通过自然语言（语音或文本）进行控制。

## 📁 项目结构

```
AI/
├── MIGRATION_GUIDE_CN.md          # 详细的迁移指南
├── INTEGRATION_STEPS_CN.md        # 完整的集成步骤
├── QUICK_START_CN.md              # 快速开始指南
├── README_MIGRATION_PROJECT.md    # 本文件
├── multimoding/                   # 源系统 (ROSMASTER M3 PRO)
│   └── Code/
│       ├── jetson nano_RaspberryPi/M3Pro_ws/
│       └── Orin/M3Pro_ws/
│           └── src/
│               ├── largemodel/    # 大模型控制包（源）
│               ├── text_chat/     # 文本聊天包（源）
│               └── interfaces/    # 消息接口（源）
└── wheeltec_ros2/                 # 目标系统 (WHEELTEC)
    └── src/
        └── largemodel_wheeltec/   # 适配的控制包（新）
            ├── largemodel/
            │   ├── action_service_wheeltec.py  # 适配的动作服务
            │   └── __init__.py
            ├── config/
            │   ├── wheeltec_config.yaml        # WHEELTEC 配置
            │   └── map_mapping.yaml            # 导航点配置
            ├── launch/
            │   └── wheeltec_voice_control.launch.py
            ├── README_CN.md           # 包使用文档
            ├── package.xml
            └── setup.py
```

## 🎯 功能特性

### 已实现功能

#### ✅ 移动控制
- 前进/后退指定距离
- 左转/右转（角度控制）
- 自定义速度控制
- 紧急停止

#### ✅ 导航功能
- 导航到预定义目标点
- 获取当前位置
- 支持中断和恢复

#### ✅ 交互功能
- 中英文双语支持
- 实时状态反馈
- 打断机制（唤醒信号）

### 已移除功能（WHEELTEC 不适用）

- ❌ 机械臂控制（arm_up, arm_down, grasp_obj 等）
- ❌ AprilTag 识别和跟踪
- ❌ 物体抓取功能
- ❌ 颜色识别跟踪

## 📖 文档指南

### 1. 快速体验 (5分钟)
👉 开始阅读: [QUICK_START_CN.md](./QUICK_START_CN.md)

适合：
- 第一次使用的用户
- 想快速验证功能的用户
- 不需要完整功能的场景

内容：
- 最简化的启动步骤
- 基础功能测试
- 常见问题快速解决

### 2. 详细迁移指南
👉 开始阅读: [MIGRATION_GUIDE_CN.md](./MIGRATION_GUIDE_CN.md)

适合：
- 需要了解系统架构的用户
- 想要理解迁移原理的开发者
- 需要自定义适配的场景

内容：
- 系统架构深度分析
- ROS2 通信机制说明
- 功能对照表
- 参数配置详解
- 故障排除指南

### 3. 完整集成步骤
👉 开始阅读: [INTEGRATION_STEPS_CN.md](./INTEGRATION_STEPS_CN.md)

适合：
- 需要完整功能的用户
- 准备生产部署的场景
- 需要语音/大模型集成

内容：
- 逐步集成指导（10个步骤）
- 大模型配置（讯飞/OpenAI/Ollama）
- 语音功能配置
- 系统优化建议
- 开机自启动配置

### 4. 包使用说明
👉 开始阅读: [wheeltec_ros2/src/largemodel_wheeltec/README_CN.md](./wheeltec_ros2/src/largemodel_wheeltec/README_CN.md)

适合：
- 日常使用参考
- API 文档查询
- 开发扩展功能

内容：
- 支持的动作函数列表
- 话题接口说明
- 配置参数详解
- 故障排除
- 扩展开发指南

## 🚀 快速开始（精简版）

### 前提条件
- WHEELTEC ROS2 环境正常
- ROS2 Humble
- 已完成 SLAM 建图

### 最小化启动（3步）

```bash
# 1. 启动 WHEELTEC 底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 2. 启动控制服务
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch largemodel_wheeltec wheeltec_voice_control.launch.py

# 3. 测试（新终端）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --once
```

详细步骤请参考 [QUICK_START_CN.md](./QUICK_START_CN.md)。

## 🔧 系统要求

### 硬件要求
- WHEELTEC 机器人（Jetson Orin NX 主控）
- （可选）USB 麦克风用于语音控制
- （可选）摄像头用于视觉功能

### 软件要求
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### Python 依赖
```bash
pip3 install pyyaml
# 如需语音功能：
pip3 install pyaudio pygame webrtcvad
```

## 📊 功能对照表

| 功能类别 | ROSMASTER M3 PRO | WHEELTEC | 说明 |
|---------|------------------|----------|------|
| 底盘移动 | ✅ | ✅ | 完全兼容 |
| 导航 | ✅ | ✅ | 基于 Nav2 |
| 语音输入 | ✅ | ✅ | 需配置麦克风 |
| 文本输入 | ✅ | ✅ | 完全兼容 |
| 大模型决策 | ✅ | ✅ | 支持多种 API |
| 机械臂 | ✅ | ❌ | WHEELTEC 无此硬件 |
| 物体识别 | ✅ | ⚠️ | 需摄像头，可选 |

## 🎓 使用场景

### 场景 1: 仓库巡检
```
用户: "巡检A区域"
机器人: 导航到A区域的各个检查点
```

### 场景 2: 物品配送
```
用户: "去仓库拿东西"
机器人: 导航到仓库位置
用户: "回到办公室"
机器人: 返回办公室
```

### 场景 3: 跟随模式
```
用户: "跟着我"
机器人: 使用传感器跟随用户
```

## 🔄 集成路线图

### 已完成 ✅
- [x] 基础移动控制适配
- [x] 导航功能集成
- [x] 配置文件创建
- [x] 文档编写
- [x] 基础测试验证

### 进行中 🚧
- [ ] 大模型服务集成
- [ ] 语音识别功能
- [ ] 完整系统测试

### 计划中 📋
- [ ] 视觉巡线集成
- [ ] 多机器人协作
- [ ] Web 控制界面
- [ ] 性能优化

## 🛠️ 开发和贡献

### 添加新动作

1. 在 `action_service_wheeltec.py` 中添加方法
2. 在 `init_language()` 添加反馈消息
3. 测试和验证
4. 更新文档

示例：
```python
def my_custom_action(self, param):
    """自定义动作"""
    # 实现逻辑
    ...
    if not self.combination_mode and not self.interrupt_flag:
        self.action_status_pub("my_action_done")
```

### 调试技巧

```bash
# 查看话题列表
ros2 topic list

# 监控速度命令
ros2 topic echo /cmd_vel

# 查看 TF 树
ros2 run tf2_tools view_frames

# 设置调试日志级别
ros2 run largemodel_wheeltec action_service --ros-args --log-level debug
```

## 📝 版本历史

### v1.0.0 (2024-11-04)
- 初始版本发布
- 完成基础移动控制适配
- 创建完整文档体系
- 支持导航和位置功能

## 📧 支持和反馈

### 获取帮助
1. 查看相关文档
2. 检查常见问题
3. 提交 GitHub Issue
4. 参考 ROS2 社区

### 报告问题
请包含：
- 系统信息（ROS2 版本、Ubuntu 版本）
- 错误日志
- 复现步骤
- 预期行为

## 📜 许可证

本项目采用 Apache-2.0 许可证。

## 🙏 致谢

- ROSMASTER M3 PRO 团队提供原始系统
- WHEELTEC 提供机器人平台
- ROS2 和 Nav2 社区
- 所有贡献者

## 📚 参考资源

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Nav2 导航文档](https://navigation.ros.org/)
- [WHEELTEC 文档](https://wheeltec.net/)
- [Python ROS2 教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

## 下一步行动

根据您的需求选择：

1. **快速体验**: 阅读 [QUICK_START_CN.md](./QUICK_START_CN.md)
2. **深入理解**: 阅读 [MIGRATION_GUIDE_CN.md](./MIGRATION_GUIDE_CN.md)
3. **完整部署**: 遵循 [INTEGRATION_STEPS_CN.md](./INTEGRATION_STEPS_CN.md)
4. **日常使用**: 参考 [largemodel_wheeltec/README_CN.md](./wheeltec_ros2/src/largemodel_wheeltec/README_CN.md)

祝您使用愉快！🎉
