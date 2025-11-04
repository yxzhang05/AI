# WHEELTEC 语音/文本控制 - 快速开始指南

## 5分钟快速体验

本指南帮助您快速体验 WHEELTEC 机器人的语音/文本控制功能。

## 前提条件

- ✅ WHEELTEC ROS2 机器人已正常工作
- ✅ ROS2 Humble 环境已配置
- ✅ 已完成基本的 SLAM 建图

## 快速开始步骤

### 第一步：准备工作空间 (2分钟)

```bash
# 进入工作空间
cd ~/wheeltec_ros2

# 确认包已存在
ls src/largemodel_wheeltec

# 如果不存在，说明还需要完成集成步骤
# 请参考 INTEGRATION_STEPS_CN.md
```

### 第二步：配置目标点 (1分钟)

编辑配置文件添加一个测试目标点：

```bash
nano src/largemodel_wheeltec/config/map_mapping.yaml
```

添加：
```yaml
test_point:
  position:
    x: 1.0    # 距离当前位置1米
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### 第三步：启动系统 (1分钟)

打开 3 个终端：

**终端 1 - 启动底盘：**
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

**终端 2 - 启动导航（如果需要导航功能）：**
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py map:=你的地图路径
```

**终端 3 - 启动控制服务：**
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch largemodel_wheeltec wheeltec_voice_control.launch.py
```

### 第四步：测试控制 (1分钟)

打开第 4 个终端，手动发送测试命令：

```bash
cd ~/wheeltec_ros2
source install/setup.bash

# 测试速度控制
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# 测试停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once
```

如果机器人响应了命令，说明基础功能工作正常！

## 仅文本模式（推荐新手）

如果暂时不需要语音功能，可以只使用文本控制：

```bash
# 终端 1-2：同上启动底盘和导航

# 终端 3：启动动作服务（文本模式）
ros2 run largemodel_wheeltec action_service --ros-args \
  -p text_chat_mode:=true \
  -p Speed_topic:=/cmd_vel \
  -p language:=zh
```

## 最简化测试（无需大模型）

如果只想测试基本的移动控制功能，无需配置大模型：

```bash
# 1. 启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 2. 在另一个终端直接发送速度命令测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

按 `Ctrl+C` 停止。

## 验证清单

完成上述步骤后，请验证：

- [ ] 机器人可以通过 `/cmd_vel` 话题控制移动
- [ ] `largemodel_wheeltec` 包可以正常启动
- [ ] 没有出现严重的错误日志

## 下一步

如果基础功能测试通过，您可以：

1. **配置大模型**: 参考 `INTEGRATION_STEPS_CN.md` 的步骤三
2. **添加更多目标点**: 编辑 `map_mapping.yaml`
3. **启用语音功能**: 参考 `INTEGRATION_STEPS_CN.md` 的步骤四
4. **完整集成**: 按照 `INTEGRATION_STEPS_CN.md` 完成所有步骤

## 常见问题

### Q: 启动时提示找不到包

**A:** 确保已编译和加载环境：
```bash
cd ~/wheeltec_ros2
colcon build --packages-select largemodel_wheeltec
source install/setup.bash
```

### Q: 机器人不动

**A:** 检查：
1. 底盘驱动是否正常启动
2. `/cmd_vel` 话题是否有订阅者：`ros2 topic info /cmd_vel`
3. 急停开关是否关闭

### Q: 想直接测试导航

**A:** 使用 rviz2 的 "2D Goal Pose" 工具：
```bash
ros2 run rviz2 rviz2 -d `ros2 pkg prefix wheeltec_robot_nav2`/share/wheeltec_robot_nav2/rviz/nav2.rviz
```

## 获取帮助

- 详细集成步骤: [INTEGRATION_STEPS_CN.md](./INTEGRATION_STEPS_CN.md)
- 完整迁移指南: [MIGRATION_GUIDE_CN.md](./MIGRATION_GUIDE_CN.md)
- 包使用说明: [wheeltec_ros2/src/largemodel_wheeltec/README_CN.md](./wheeltec_ros2/src/largemodel_wheeltec/README_CN.md)

## 成功案例

完成快速开始后，您应该能够：
- ✅ 通过话题控制机器人移动
- ✅ 理解系统的基本架构
- ✅ 为完整集成做好准备

继续前往 `INTEGRATION_STEPS_CN.md` 完成完整的语音/文本控制系统集成！
