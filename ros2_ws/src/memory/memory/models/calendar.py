#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
日历事件模型

管理用户的日程安排、重要事件等
"""

from datetime import datetime

class CalendarEvent:
    """日历事件类"""
    
    def __init__(self, event_id, user_id, title, start_time, end_time=None, 
                 recurrence=None, reminder=None, description=None):
        """
        初始化日历事件
        
        Args:
            event_id (str): 事件唯一标识
            user_id (str): 用户唯一标识
            title (str): 事件标题
            start_time (datetime): 开始时间
            end_time (datetime, optional): 结束时间
            recurrence (str, optional): 重复规则
            reminder (dict, optional): 提醒设置
            description (str, optional): 事件描述
        """
        self.event_id = event_id
        self.user_id = user_id
        self.title = title
        self.start_time = start_time
        self.end_time = end_time
        self.recurrence = recurrence  # 如：daily, weekly, monthly等
        self.reminder = reminder      # 如：{"minutes": 30, "method": "notification"}
        self.description = description
        self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def to_dict(self):
        """
        转换为字典格式
        
        Returns:
            dict: 日历事件字典
        """
        return {
            'event_id': self.event_id,
            'user_id': self.user_id,
            'title': self.title,
            'start_time': self.start_time.isoformat(),
            'end_time': self.end_time.isoformat() if self.end_time else None,
            'recurrence': self.recurrence,
            'reminder': self.reminder,
            'description': self.description,
            'created_at': self.created_at.isoformat(),
            'updated_at': self.updated_at.isoformat()
        }
    
    @classmethod
    def from_dict(cls, data):
        """
        从字典创建日历事件
        
        Args:
            data (dict): 日历事件字典
            
        Returns:
            CalendarEvent: 日历事件对象
        """
        event = cls(
            event_id=data['event_id'],
            user_id=data['user_id'],
            title=data['title'],
            start_time=datetime.fromisoformat(data['start_time']),
            end_time=datetime.fromisoformat(data['end_time']) if data.get('end_time') else None,
            recurrence=data.get('recurrence'),
            reminder=data.get('reminder'),
            description=data.get('description')
        )
        if 'created_at' in data:
            event.created_at = datetime.fromisoformat(data['created_at'])
        if 'updated_at' in data:
            event.updated_at = datetime.fromisoformat(data['updated_at'])
        return event
    
    def update(self, **kwargs):
        """
        更新日历事件
        
        Args:
            **kwargs: 要更新的属性
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.updated_at = datetime.now()
    
    def is_upcoming(self, hours=24):
        """
        检查事件是否即将到来
        
        Args:
            hours (int, optional): 检查时间范围（小时）
            
        Returns:
            bool: 是否即将到来
        """
        now = datetime.now()
        time_diff = self.start_time - now
        return 0 <= time_diff.total_seconds() <= hours * 3600
    
    def __str__(self):
        """
        字符串表示
        
        Returns:
            str: 日历事件字符串
        """
        return f"CalendarEvent(event_id={self.event_id}, title={self.title}, start_time={self.start_time})"
