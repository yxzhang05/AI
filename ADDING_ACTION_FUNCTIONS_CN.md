# 为 WHEELTEC 机器人添加新动作函数指南

## 概述

本指南说明如何为 WHEELTEC 机器人的动作服务添加新的动作函数，使大模型能够调用这些功能。

## 问题背景

用户尝试通过文本输入控制视觉跟随功能时，收到错误：

```
[INFO] [action_service_ndoe]: Published message: 机器人反馈:动作函数不存在，无法执行
```

这是因为 action_service 中还没有定义视觉跟随、视觉巡线和导航等高级功能的动作函数。

## 系统架构回顾

```
用户输入 → text_chat → /asr → model_service → action_service → 功能启动
                         ↑                           ↓
                         └──── /text_response ───────┘
```

当用户输入"视觉跟随红色物体"时：
1. text_chat 发送到 /asr
2. model_service 解析意图，调用对应函数（如 `start_visual_follower()`）
3. action_service 执行函数（启动相应的 launch 文件）
4. 返回执行状态

## 添加新动作函数的步骤

### 第一步：在 action_service 中添加函数定义

需要编辑的文件位置（根据您的系统）：
- **源代码位置**：`~/wheeltec_ros2/src/largemodel/largemodel/action_service.py`
- **安装位置**：`~/wheeltec_ros2/install/largemodel/lib/python3.10/site-packages/largemodel/action_service.py`

**建议**：修改源代码位置，然后重新编译。

#### 1.1 添加视觉跟随功能

在 action_service.py 的类中添加以下方法：

```python
def start_visual_follower(self, color="red"):
    """
    启动视觉跟随功能
    Start visual follower functionality
    
    Args:
        color: 要跟随的颜色 (red, blue, green, yellow 等)
    """
    import subprocess
    
    self.get_logger().info(f"启动视觉跟随功能，跟随颜色：{color}")
    
    try:
        # 启动视觉跟随 launch 文件
        # 注意：这是后台启动，需要管理进程
        cmd = ["ros2", "launch", "simple_follower_ros2", "visual_follower.launch.py"]
        
        # 如果需要传递参数（如颜色），可以添加：
        # cmd.extend([f"target_color:={color}"])
        
        # 启动进程并保存引用（如果需要后续停止）
        self.visual_follower_process = subprocess.Popen(cmd)
        
        # 等待一小段时间确保启动
        import time
        time.sleep(2)
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_visual_follower_done", color=color)
            
    except Exception as e:
        self.get_logger().error(f"启动视觉跟随失败: {e}")
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_visual_follower_failed", color=color)

def stop_visual_follower(self):
    """
    停止视觉跟随功能
    Stop visual follower functionality
    """
    self.get_logger().info("停止视觉跟随功能")
    
    try:
        if hasattr(self, 'visual_follower_process'):
            self.visual_follower_process.terminate()
            self.visual_follower_process.wait(timeout=5)
            delattr(self, 'visual_follower_process')
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("stop_visual_follower_done")
            
    except Exception as e:
        self.get_logger().error(f"停止视觉跟随失败: {e}")
```

#### 1.2 添加视觉巡线功能

```python
def start_line_follower(self):
    """
    启动视觉巡线功能
    Start line follower functionality
    """
    import subprocess
    
    self.get_logger().info("启动视觉巡线功能")
    
    try:
        cmd = ["ros2", "launch", "simple_follower_ros2", "line_follower.launch.py"]
        self.line_follower_process = subprocess.Popen(cmd)
        
        import time
        time.sleep(2)
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_line_follower_done")
            
    except Exception as e:
        self.get_logger().error(f"启动视觉巡线失败: {e}")
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_line_follower_failed")

def stop_line_follower(self):
    """
    停止视觉巡线功能
    Stop line follower functionality
    """
    self.get_logger().info("停止视觉巡线功能")
    
    try:
        if hasattr(self, 'line_follower_process'):
            self.line_follower_process.terminate()
            self.line_follower_process.wait(timeout=5)
            delattr(self, 'line_follower_process')
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("stop_line_follower_done")
            
    except Exception as e:
        self.get_logger().error(f"停止视觉巡线失败: {e}")
```

#### 1.3 添加导航功能启动/停止

```python
def start_navigation_system(self):
    """
    启动导航系统
    Start navigation system
    """
    import subprocess
    
    self.get_logger().info("启动导航系统")
    
    try:
        cmd = ["ros2", "launch", "wheeltec_nav2", "wheeltec_nav2.launch.py"]
        self.nav_system_process = subprocess.Popen(cmd)
        
        import time
        time.sleep(3)  # 导航系统需要更长启动时间
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_navigation_system_done")
            
    except Exception as e:
        self.get_logger().error(f"启动导航系统失败: {e}")
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("start_navigation_system_failed")

def stop_navigation_system(self):
    """
    停止导航系统
    Stop navigation system
    """
    self.get_logger().info("停止导航系统")
    
    try:
        if hasattr(self, 'nav_system_process'):
            self.nav_system_process.terminate()
            self.nav_system_process.wait(timeout=5)
            delattr(self, 'nav_system_process')
        
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("stop_navigation_system_done")
            
    except Exception as e:
        self.get_logger().error(f"停止导航系统失败: {e}")
```

#### 1.4 在 __init__ 方法中初始化进程引用

在 `__init__` 方法中添加：

```python
def __init__(self):
    super().__init__("action_service_node")
    
    # ... 其他初始化代码 ...
    
    # 初始化进程引用
    self.visual_follower_process = None
    self.line_follower_process = None
    self.nav_system_process = None
```

### 第二步：添加反馈消息

在 `init_language()` 方法中添加反馈消息：

```python
def init_language(self):
    """初始化语言设置"""
    # ... 现有代码 ...
    
    self.feedback_dict = {
        "zh": {
            # ... 现有的反馈消息 ...
            
            # 新增视觉跟随反馈
            "start_visual_follower_done": "机器人反馈:已启动视觉跟随功能，正在跟随{color}色物体",
            "start_visual_follower_failed": "机器人反馈:启动视觉跟随失败",
            "stop_visual_follower_done": "机器人反馈:已停止视觉跟随",
            
            # 新增视觉巡线反馈
            "start_line_follower_done": "机器人反馈:已启动视觉巡线功能",
            "start_line_follower_failed": "机器人反馈:启动视觉巡线失败",
            "stop_line_follower_done": "机器人反馈:已停止视觉巡线",
            
            # 新增导航系统反馈
            "start_navigation_system_done": "机器人反馈:导航系统已启动",
            "start_navigation_system_failed": "机器人反馈:导航系统启动失败",
            "stop_navigation_system_done": "机器人反馈:导航系统已停止",
        },
        "en": {
            # ... 现有的英文反馈 ...
            
            # Visual follower feedback
            "start_visual_follower_done": "Robot feedback: Visual follower started, following {color} object",
            "start_visual_follower_failed": "Robot feedback: Failed to start visual follower",
            "stop_visual_follower_done": "Robot feedback: Visual follower stopped",
            
            # Line follower feedback
            "start_line_follower_done": "Robot feedback: Line follower started",
            "start_line_follower_failed": "Robot feedback: Failed to start line follower",
            "stop_line_follower_done": "Robot feedback: Line follower stopped",
            
            # Navigation system feedback
            "start_navigation_system_done": "Robot feedback: Navigation system started",
            "start_navigation_system_failed": "Robot feedback: Failed to start navigation system",
            "stop_navigation_system_done": "Robot feedback: Navigation system stopped",
        },
    }
```

### 第三步：重新编译包

```bash
cd ~/wheeltec_ros2
colcon build --packages-select largemodel
source install/setup.bash
```

### 第四步：配置大模型服务的函数定义

大模型服务需要知道这些新函数的存在。需要在大模型服务的配置或代码中添加函数定义。

#### 方式 1：在 model_service 中添加函数描述（推荐）

编辑 `model_service.py`，在函数列表中添加：

```python
functions = [
    # ... 现有函数定义 ...
    
    {
        "name": "start_visual_follower",
        "description": "启动视觉跟随功能，让机器人跟随指定颜色的物体移动",
        "parameters": {
            "type": "object",
            "properties": {
                "color": {
                    "type": "string",
                    "enum": ["red", "blue", "green", "yellow"],
                    "description": "要跟随的物体颜色",
                    "default": "red"
                }
            }
        }
    },
    {
        "name": "stop_visual_follower",
        "description": "停止视觉跟随功能"
    },
    {
        "name": "start_line_follower",
        "description": "启动视觉巡线功能，让机器人沿着地面上的线条移动"
    },
    {
        "name": "stop_line_follower",
        "description": "停止视觉巡线功能"
    },
    {
        "name": "start_navigation_system",
        "description": "启动导航系统，准备进行自主导航"
    },
    {
        "name": "stop_navigation_system",
        "description": "停止导航系统"
    }
]
```

#### 方式 2：通过 Prompt 让大模型知道可用函数

在大模型的 system prompt 中添加：

```
你可以调用以下机器人控制函数：
...（现有函数）...

高级功能函数：
- start_visual_follower(color): 启动视觉跟随，跟随指定颜色的物体
- stop_visual_follower(): 停止视觉跟随
- start_line_follower(): 启动视觉巡线
- stop_line_follower(): 停止视觉巡线
- start_navigation_system(): 启动导航系统
- stop_navigation_system(): 停止导航系统
```

## 完整的代码示例

### action_service.py 完整添加示例

```python
class ActionService(Node):
    def __init__(self):
        super().__init__("action_service_node")
        # ... 现有初始化代码 ...
        
        # 初始化高级功能进程引用
        self.visual_follower_process = None
        self.line_follower_process = None
        self.nav_system_process = None
    
    # ... 现有方法 ...
    
    # ========== 新增高级功能方法 ==========
    
    def start_visual_follower(self, color="red"):
        """启动视觉跟随功能"""
        import subprocess
        import time
        
        self.get_logger().info(f"启动视觉跟随功能，跟随颜色：{color}")
        
        try:
            # 如果已经在运行，先停止
            if self.visual_follower_process is not None:
                self.stop_visual_follower()
            
            # 启动新的视觉跟随进程
            cmd = ["ros2", "launch", "simple_follower_ros2", "visual_follower.launch.py"]
            self.visual_follower_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 等待启动
            time.sleep(2)
            
            # 检查进程是否还在运行
            if self.visual_follower_process.poll() is None:
                self.get_logger().info("视觉跟随启动成功")
                if not self.combination_mode and not self.interrupt_flag:
                    self.action_status_pub("start_visual_follower_done", color=color)
            else:
                raise Exception("进程启动后立即退出")
                
        except Exception as e:
            self.get_logger().error(f"启动视觉跟随失败: {e}")
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("start_visual_follower_failed", color=color)
    
    def stop_visual_follower(self):
        """停止视觉跟随功能"""
        self.get_logger().info("停止视觉跟随功能")
        
        try:
            if self.visual_follower_process is not None:
                self.visual_follower_process.terminate()
                try:
                    self.visual_follower_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.visual_follower_process.kill()
                self.visual_follower_process = None
            
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("stop_visual_follower_done")
                
        except Exception as e:
            self.get_logger().error(f"停止视觉跟随失败: {e}")
    
    def start_line_follower(self):
        """启动视觉巡线功能"""
        import subprocess
        import time
        
        self.get_logger().info("启动视觉巡线功能")
        
        try:
            if self.line_follower_process is not None:
                self.stop_line_follower()
            
            cmd = ["ros2", "launch", "simple_follower_ros2", "line_follower.launch.py"]
            self.line_follower_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            if self.line_follower_process.poll() is None:
                self.get_logger().info("视觉巡线启动成功")
                if not self.combination_mode and not self.interrupt_flag:
                    self.action_status_pub("start_line_follower_done")
            else:
                raise Exception("进程启动后立即退出")
                
        except Exception as e:
            self.get_logger().error(f"启动视觉巡线失败: {e}")
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("start_line_follower_failed")
    
    def stop_line_follower(self):
        """停止视觉巡线功能"""
        self.get_logger().info("停止视觉巡线功能")
        
        try:
            if self.line_follower_process is not None:
                self.line_follower_process.terminate()
                try:
                    self.line_follower_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.line_follower_process.kill()
                self.line_follower_process = None
            
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("stop_line_follower_done")
                
        except Exception as e:
            self.get_logger().error(f"停止视觉巡线失败: {e}")
    
    def start_navigation_system(self):
        """启动导航系统"""
        import subprocess
        import time
        
        self.get_logger().info("启动导航系统")
        
        try:
            if self.nav_system_process is not None:
                self.stop_navigation_system()
            
            cmd = ["ros2", "launch", "wheeltec_nav2", "wheeltec_nav2.launch.py"]
            self.nav_system_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(3)  # 导航系统需要更长启动时间
            
            if self.nav_system_process.poll() is None:
                self.get_logger().info("导航系统启动成功")
                if not self.combination_mode and not self.interrupt_flag:
                    self.action_status_pub("start_navigation_system_done")
            else:
                raise Exception("进程启动后立即退出")
                
        except Exception as e:
            self.get_logger().error(f"启动导航系统失败: {e}")
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("start_navigation_system_failed")
    
    def stop_navigation_system(self):
        """停止导航系统"""
        self.get_logger().info("停止导航系统")
        
        try:
            if self.nav_system_process is not None:
                self.nav_system_process.terminate()
                try:
                    self.nav_system_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.nav_system_process.kill()
                self.nav_system_process = None
            
            if not self.combination_mode and not self.interrupt_flag:
                self.action_status_pub("stop_navigation_system_done")
                
        except Exception as e:
            self.get_logger().error(f"停止导航系统失败: {e}")
    
    # ========== 清理方法 ==========
    
    def __del__(self):
        """析构函数，确保所有进程被清理"""
        try:
            if hasattr(self, 'visual_follower_process') and self.visual_follower_process:
                self.visual_follower_process.terminate()
            if hasattr(self, 'line_follower_process') and self.line_follower_process:
                self.line_follower_process.terminate()
            if hasattr(self, 'nav_system_process') and self.nav_system_process:
                self.nav_system_process.terminate()
        except:
            pass
```

## 使用示例

添加并配置好这些函数后，用户就可以通过文本输入控制：

### 视觉跟随
```
用户：开始视觉跟随红色物体
机器人：机器人反馈:已启动视觉跟随功能，正在跟随red色物体

用户：停止视觉跟随
机器人：机器人反馈:已停止视觉跟随
```

### 视觉巡线
```
用户：开始视觉巡线
机器人：机器人反馈:已启动视觉巡线功能

用户：停止巡线
机器人：机器人反馈:已停止视觉巡线
```

### 导航系统
```
用户：启动导航系统
机器人：机器人反馈:导航系统已启动

用户：导航到客厅
机器人：机器人反馈:执行navigation(living_room)完成

用户：关闭导航系统
机器人：机器人反馈:导航系统已停止
```

## 注意事项

### 1. 进程管理

使用 `subprocess.Popen` 启动的进程需要妥善管理：
- ✅ 保存进程引用以便后续停止
- ✅ 在停止时使用 `terminate()` 而不是 `kill()`
- ✅ 设置超时避免无限等待
- ✅ 在节点销毁时清理所有子进程

### 2. 互斥性

某些功能可能互斥，需要先停止一个才能启动另一个：
- 视觉跟随和视觉巡线可能不能同时运行
- 建议在启动新功能前先检查并停止旧功能

### 3. 状态管理

可以添加状态标志来跟踪哪些功能正在运行：

```python
self.running_modes = {
    'visual_follower': False,
    'line_follower': False,
    'navigation': False
}
```

### 4. 错误处理

确保捕获所有可能的异常：
- launch 文件不存在
- 权限问题
- 进程启动失败
- 超时

### 5. 参数传递

如果需要向 launch 文件传递参数：

```python
cmd = [
    "ros2", "launch", 
    "simple_follower_ros2", "visual_follower.launch.py",
    f"target_color:={color}",
    f"camera_topic:=/camera/image_raw"
]
```

## 测试步骤

1. **编译代码**
   ```bash
   cd ~/wheeltec_ros2
   colcon build --packages-select largemodel
   source install/setup.bash
   ```

2. **启动服务**
   ```bash
   # 终端 1: 底盘
   ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
   
   # 终端 2: action_service
   ros2 run largemodel action_service --ros-args --params-file src/largemodel/config/wheeltec_config.yaml
   
   # 终端 3: model_service
   ros2 run largemodel model_service --ros-args --params-file src/largemodel/config/wheeltec_config.yaml
   
   # 终端 4: text_chat
   ros2 run text_chat text_chat
   ```

3. **测试命令**
   - "开始视觉跟随"
   - "停止视觉跟随"
   - "开始视觉巡线"
   - "停止巡线"
   - "启动导航系统"

4. **验证**
   - 检查 action_service 日志是否有启动信息
   - 使用 `ros2 node list` 查看是否有新节点启动
   - 观察机器人实际行为

## 故障排除

### 问题1: 函数仍然不存在

**检查**：
- action_service.py 中是否添加了函数
- 是否重新编译了包
- 是否 source 了新的环境
- model_service 是否知道这些函数（检查函数列表或 prompt）

### 问题2: 进程启动失败

**检查**：
- launch 文件路径是否正确
- 相关包是否已安装（simple_follower_ros2, wheeltec_nav2）
- 是否有权限问题
- 查看 action_service 的错误日志

### 问题3: 进程无法停止

**解决**：
- 使用 `ps aux | grep ros2` 查找进程
- 手动 kill: `kill -9 <pid>`
- 改进代码中的进程清理逻辑

## 更进一步的改进

### 1. 使用 ROS2 Launch API

更优雅的方式是使用 ROS2 的 Python Launch API：

```python
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def start_visual_follower_advanced(self):
    launch_service = LaunchService()
    # ... 配置 launch
    launch_service.run()
```

### 2. 使用 ROS2 Lifecycle 节点

更好的状态管理和控制。

### 3. 添加状态查询功能

```python
def get_system_status(self):
    """查询当前运行的功能"""
    status = []
    if self.visual_follower_process:
        status.append("视觉跟随")
    if self.line_follower_process:
        status.append("视觉巡线")
    # ...
    return "当前运行: " + ", ".join(status) if status else "当前无高级功能运行"
```

## 总结

通过以上步骤，您可以：
1. ✅ 在 action_service 中添加新的动作函数
2. ✅ 配置大模型服务识别这些函数
3. ✅ 通过文本输入控制 WHEELTEC 的高级功能
4. ✅ 妥善管理子进程的生命周期

**关键点**：
- 在 action_service.py 中定义函数
- 添加中英文反馈消息
- 在 model_service 中注册函数
- 重新编译并测试

---

**版本**: 1.0  
**适用于**: WHEELTEC ROS2 Humble with Large Model Service  
**最后更新**: 2024-11-21
