# ROS2机器人系统

这是基于ROS2的智能陪伴机器人系统，包含情感交互和视觉感知等功能模块。

## 目录结构

```
ros2_ws/
├── src/
│   ├── emotion_interaction/       # 情感交互模块
│   ├── emotion_msgs/              # 情感消息定义
│   ├── robot_bringup/             # 机器人启动包
│   ├── vision_perception/         # 视觉感知模块
│   ├── vision_msgs/               # 视觉消息定义
│   └── voice_msgs/                # 语音消息定义
```

## 环境要求

- ROS2 Humble (或更高版本)
- Python 3.8+ 
- rclpy
- 其他依赖库

## 安装和运行

### 1. 安装依赖

```bash
# 安装ROS2 Humble
# 参考：https://docs.ros.org/en/humble/Installation.html

# 安装Python依赖
pip3 install rclpy
```

### 2. 构建工作空间

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 运行机器人系统

```bash
# 启动所有模块
ros2 launch robot_bringup robot_launch.py

# 只启动情感交互模块
ros2 launch robot_bringup emotion_launch.py

# 只启动视觉感知模块  
ros2 launch robot_bringup vision_launch.py

# 不启动视觉模块
ros2 launch robot_bringup robot_launch.py enable_vision:=false
```

### 4. 查看节点和话题

```bash
# 查看运行中的节点
ros2 node list

# 查看话题
ros2 topic list

# 查看语音命令话题内容
ros2 topic echo /voice_command

# 查看情感状态话题内容
ros2 topic echo /emotion_state
```

## 模块说明

### 情感交互模块

- **voice_recognition_node**: 语音识别节点，发布语音命令
- **emotion_engine_node**: 情感引擎节点，处理情感分析
- **text_to_speech_node**: 语音合成节点，提供语音合成服务

### 视觉感知模块

- **face_detection_node**: 人脸识别节点，发布人脸识别结果

## 消息定义

### 语音消息 (voice_msgs)

- `VoiceCommand.msg`: 语音命令消息
- `TextToSpeech.srv`: 语音合成服务

### 情感消息 (emotion_msgs)

- `EmotionState.msg`: 情感状态消息

### 视觉消息 (vision_msgs)

- `FaceDetection.msg`: 人脸识别结果消息
- `ObjectDetection.msg`: 目标检测结果消息
- `RecognizeFace.srv`: 人脸识别服务
- `DetectObjects.srv`: 目标检测服务

## 开发说明

### 创建新节点

1. 在对应的功能包中创建新的节点文件
2. 在setup.py中添加入口点
3. 在启动文件中添加节点配置

### 测试节点

```bash
# 运行单个节点
ros2 run emotion_interaction voice_recognition_node

# 运行通信测试
python3 src/emotion_interaction/test_communication.py
```

## 注意事项

1. 确保已正确安装ROS2环境
2. 每次修改代码后需要重新构建工作空间
3. 启动前确保已正确设置环境变量

## 故障排除

- **问题**: 找不到包
  **解决方案**: 确保已正确构建工作空间并source setup.bash

- **问题**: 节点无法启动
  **解决方案**: 检查节点代码是否有错误，查看终端输出的错误信息

- **问题**: 话题没有消息
  **解决方案**: 检查节点是否正常运行，使用ros2 node list和ros2 topic list查看状态

## 许可证

Apache License 2.0
