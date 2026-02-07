#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
数据模型模块

定义用户档案、偏好设置、日历事件等数据模型
"""

# 使用相对导入
from .user_profile import UserProfile
from .preference import Preference
from .calendar import CalendarEvent
from .reminder import ReminderRecord

__all__ = [
    'UserProfile',
    'Preference',
    'CalendarEvent',
    'ReminderRecord'
]
