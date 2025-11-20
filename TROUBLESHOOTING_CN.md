# WHEELTEC 文本输入功能调试指南

## 问题描述

用户报告：文本输入和大模型服务都正常响应，但小车没有实际移动。

## 系统架构回顾

```
用户输入 → text_chat → /asr → model_service → action_service → /cmd_vel → 底盘驱动
                         ↑                           ↓
                         └──── /text_response ────────┘
```

## 调试步骤

### 第一步：确认各节点是否正常运行

在新终端执行：

```bash
# 查看所有运行的 ROS2 节点
ros2 node list
```

**预期输出应包含**：
- `wheeltec_action_service` 或类似的 action_service 节点
- 大模型服务节点
- text_chat 节点
- wheeltec 底盘驱动节点

**如果缺少任何节点，说明该组件未正常启动。**

---

### 第二步：检查话题发布情况

#### 2.1 检查 /asr 话题（用户输入）

```bash
# 监听 /asr 话题
ros2 topic echo /asr
```

然后在 text_chat 终端输入指令，查看是否有消息发布。

**预期**：看到用户输入的文本消息。

#### 2.2 检查 /cmd_vel 话题（速度控制）

```bash
# 监听 /cmd_vel 话题
ros2 topic echo /cmd_vel
```

当用户输入移动指令后，这里应该看到速度命令。

**预期**：看到类似这样的消息：
```
linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

**⚠️ 关键判断点**：
- **如果没有看到 /cmd_vel 消息**，说明 action_service 没有正常执行动作
- **如果看到了 /cmd_vel 消息**，说明问题在底盘驱动层

---

### 第三步：诊断 action_service

#### 3.1 检查 action_service 是否正确订阅话题

```bash
# 查看 action_service 节点的订阅和发布
ros2 node info /wheeltec_action_service
```

**预期应该看到**：
- Publishers（发布者）：`/cmd_vel`, `/actionstatus`
- Subscribers（订阅者）：`/wakeup`

**注意**：action_service 并不直接订阅 /asr 话题，它是通过大模型服务的函数调用来执行动作的。

#### 3.2 检查 action_service 日志

查看 action_service 启动终端的输出，是否有：
- 函数调用日志
- 错误信息
- 异常提示

---

### 第四步：检查大模型服务的函数调用

这是最关键的一步！大模型服务需要正确地调用 action_service 中的函数。

#### 4.1 查看大模型服务日志

在运行 `model_service` 的终端查看：
1. 是否收到 /asr 消息？
2. 是否解析了用户意图？
3. **是否实际调用了 Python 函数**？

**关键问题**：大模型可能只是生成了文本回复，而没有调用 move_forward() 函数。

#### 4.2 确认函数调用接口配置

检查 `wheeltec_config.yaml` 中的配置是否正确：

```bash
cd ~/wheeltec_ros2/src/largemodel/config
cat wheeltec_config.yaml
```

确认 `regional_setting` 和 `language` 等参数正确。

---

### 第五步：手动测试 action_service

绕过大模型服务，直接测试 action_service 和底盘驱动：

#### 5.1 手动发布速度命令

```bash
# 测试前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 等待 2 秒后停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

**结果判断**：
- **如果小车移动了** → 底盘驱动正常，问题在 action_service 或大模型服务
- **如果小车不移动** → 底盘驱动有问题，需要检查 wheeltec 底层

#### 5.2 检查底盘驱动话题订阅

```bash
# 查看哪些节点订阅了 /cmd_vel
ros2 topic info /cmd_vel
```

**预期**：应该看到 wheeltec 底盘驱动节点订阅了这个话题。

**如果没有订阅者**：
- 底盘驱动可能没有启动
- 或者订阅的话题名称不对

---

### 第六步：检查话题名称是否匹配

#### 6.1 确认 action_service 发布到正确的话题

检查 `wheeltec_config.yaml`：

```yaml
action_service:
  ros__parameters:
    Speed_topic: "/cmd_vel"  # 这里的话题名称必须正确
```

#### 6.2 确认底盘驱动订阅的话题名称

查看底盘驱动节点信息：

```bash
# 找到 wheeltec 底盘驱动节点（名称可能不同）
ros2 node list | grep wheeltec

# 查看该节点的订阅
ros2 node info /wheeltec_driver  # 替换为实际节点名
```

**如果话题名称不匹配**：
- 修改 `wheeltec_config.yaml` 中的 `Speed_topic` 参数
- 或者检查底盘驱动的配置

---

## 常见问题诊断

### 问题 1：大模型响应了，但 action_service 没有执行

**可能原因**：
1. 大模型只生成了文本回复，没有调用函数
2. 函数调用格式不正确
3. action_service 没有正确注册函数接口

**解决方案**：
- 检查大模型服务的函数调用配置（参考 course 教程）
- 确认 action_service 中的函数接口定义正确
- 查看大模型服务日志，确认是否有函数调用输出

---

### 问题 2：action_service 执行了，但小车不动

**可能原因**：
1. `/cmd_vel` 话题名称不匹配
2. 底盘驱动没有启动或订阅错误的话题
3. 底盘电源或连接问题

**解决方案**：
1. 使用手动发布测试（见第五步）
2. 检查话题名称配置
3. 检查 wheeltec 底盘硬件状态

---

### 问题 3：/cmd_vel 有消息，但小车不动

**可能原因**：
1. 底盘驱动软件问题
2. 底盘硬件未启动或故障
3. 安全保护机制触发
4. 电池电量不足

**解决方案**：
1. 重启底盘驱动：
   ```bash
   # 终端 1：Ctrl+C 停止，然后重新启动
   ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
   ```
2. 检查底盘指示灯状态
3. 检查电池电量
4. 检查是否有急停开关

---

## 调试命令速查表

```bash
# 1. 查看所有节点
ros2 node list

# 2. 查看所有话题
ros2 topic list

# 3. 查看特定话题的信息
ros2 topic info /cmd_vel

# 4. 监听话题内容
ros2 topic echo /asr
ros2 topic echo /cmd_vel
ros2 topic echo /actionstatus
ros2 topic echo /text_response

# 5. 查看话题发布频率
ros2 topic hz /cmd_vel

# 6. 查看节点详细信息
ros2 node info /wheeltec_action_service

# 7. 手动发布测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# 8. 查看 ROS2 网络连接
ros2 daemon status
```

---

## 推荐的调试流程

按照以下顺序逐步排查：

1. ✅ **确认所有节点已启动** → `ros2 node list`
2. ✅ **确认 /asr 话题有消息** → `ros2 topic echo /asr`
3. ✅ **确认大模型服务响应** → 查看 model_service 日志
4. ❓ **确认大模型调用了函数** → **这是关键！**
5. ❓ **确认 /cmd_vel 有消息** → `ros2 topic echo /cmd_vel`
6. ❓ **手动测试底盘** → 手动发布 /cmd_vel
7. ✅ **排查硬件问题** → 检查底盘、电源、连接

---

## 最可能的原因

根据您的描述"大模型也对此命令进行了响应"，但小车不动，**最可能的原因是**：

### 🔴 大模型只生成了文本回复，没有调用 action_service 的函数

**验证方法**：
```bash
# 监听 /cmd_vel 话题
ros2 topic echo /cmd_vel
```

然后输入指令，如果：
- **看不到任何消息** → 大模型没有调用函数
- **看到消息但小车不动** → 底盘驱动问题

**解决方案**（如果是函数调用问题）：
1. 检查大模型服务的函数调用配置
2. 确认 action_service 中的函数是否正确注册
3. 查看大模型服务的日志，看是否有函数调用相关的输出
4. 参考 `course` 文件夹中的教程，确认函数调用接口配置正确

---

## 需要提供的调试信息

如果问题仍未解决，请提供以下信息：

1. `ros2 node list` 的完整输出
2. `ros2 topic list` 的完整输出
3. `ros2 topic echo /cmd_vel` 的输出（输入指令后）
4. action_service 启动终端的完整日志
5. model_service 启动终端的完整日志
6. 手动发布 /cmd_vel 后小车是否移动

---

## 快速诊断脚本

保存以下内容为 `debug_wheeltec.sh`，然后运行 `bash debug_wheeltec.sh`：

```bash
#!/bin/bash
echo "========== WHEELTEC 调试信息 =========="
echo ""
echo "1. ROS2 节点列表："
ros2 node list
echo ""
echo "2. ROS2 话题列表："
ros2 topic list | grep -E "(asr|cmd_vel|actionstatus|text_response)"
echo ""
echo "3. /cmd_vel 话题信息："
ros2 topic info /cmd_vel
echo ""
echo "4. 测试手动发布 /cmd_vel (小车应该会短暂移动)："
echo "发送前进指令..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
sleep 2
echo "发送停止指令..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once
echo ""
echo "========== 调试完成 =========="
echo "如果小车在步骤4移动了，说明底盘驱动正常"
echo "如果没有移动，请检查底盘驱动和硬件连接"
```

---

**祝调试顺利！** 🚀

如有其他问题，请提供上述调试信息以便进一步分析。
