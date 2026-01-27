from enum import Enum


class MessageType(Enum):
    """消息类型枚举类，定义了系统中使用的各种MQTT消息类型"""
    HELLO = "hello"          # 会话建立消息，用于初始化连接和会话
    TTS = "tts"              # 文本转语音消息，用于发送文本到语音服务
    STT = "stt"              # 语音转文本消息，用于发送语音到文本服务
    LLM = "llm"              # 大语言模型消息，用于与LLM服务交互
    GOODBYE = "goodbye"      # 会话结束消息，用于关闭连接和会话
    HEARTBEAT = "heartbeat"  # 心跳消息，用于维持连接状态