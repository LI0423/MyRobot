# "小陪"智能陪伴机器人

## 项目概述
基于RDK X5开发的一款面向家庭用户的智能陪伴机器人，专注于提供情感陪伴、生活辅助和娱乐互动功能。

## 核心功能
- 情感交互：语音对话、表情识别、情感表达
- 视觉感知：人脸识别、场景理解、运动追踪
- 生活辅助：日程提醒、健康监测、用药提醒
- 远程监控：实时视频、双向语音、异常报警
- 娱乐互动：故事播放、音乐播放、游戏互动
- 家居控制：智能家电控制、环境监测、场景模式

## 技术架构
- **硬件平台**：RDK X5机器人开发套件
- **操作系统**：Ubuntu 22.04 LTS + Robot OS
- **AI框架**：TensorFlow、PyTorch
- **编程语言**：Python、C++

## 项目结构
```
MyRobot/
├── config/          # 配置文件目录
├── docs/            # 文档目录
├── hardware/        # 硬件相关代码
├── modules/         # 功能模块
│   ├── emotion/     # 情感交互模块
│   ├── vision/      # 视觉感知模块
│   ├── life/        # 生活辅助模块
│   ├── monitor/     # 远程监控模块
│   ├── entertainment/ # 娱乐互动模块
│   └── home/        # 家居控制模块
├── scripts/         # 脚本文件
├── tests/           # 测试代码
├── utils/           # 工具函数
├── main.py          # 主程序入口
├── requirements.txt # 依赖包
└── README.md        # 项目说明
```

## 开发环境搭建
1. 安装Ubuntu 22.04 LTS
2. 安装Robot OS
3. 安装AI框架（TensorFlow、PyTorch）
4. 安装依赖包：`pip install -r requirements.txt`

## 运行项目
```bash
python main.py
```

## 文档
- 产品规划文档：`docs/产品规划文档.md`
- 需求文档：`docs/需求文档.md`
- 技术文档：`docs/技术文档.md`

## 开发团队
产品团队

## 版本历史
- v1.0.0：初始版本
