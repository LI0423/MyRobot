#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
记忆模块测试脚本

测试个性化记忆库的功能
"""

import os
import sys
import uuid
from datetime import datetime, timedelta

# 导入记忆模块
from memory.memory.memory_manager import MemoryManager
from memory.memory.models.user_profile import UserProfile
from memory.memory.models.preference import Preference
from memory.memory.models.calendar import CalendarEvent

def test_memory_module():
    """
    测试记忆模块功能
    """
    print("=== 测试个性化记忆库 ===")
    
    # 初始化记忆管理器
    memory_manager = MemoryManager(db_path='./test_memory.db')
    print("✓ 记忆管理器初始化成功")
    
    # 生成测试用户ID
    test_user_id = str(uuid.uuid4())
    print(f"测试用户ID: {test_user_id}")
    
    # 1. 测试用户档案管理
    print("\n1. 测试用户档案管理")
    
    # 创建用户档案
    profile = UserProfile(
        user_id=test_user_id,
        name="测试用户",
        age=30,
        gender="男",
        relationship="主人"
    )
    memory_manager.create_user_profile(profile)
    print("✓ 创建用户档案成功")
    
    # 获取用户档案
    retrieved_profile = memory_manager.get_user_profile(test_user_id)
    if retrieved_profile:
        print(f"✓ 获取用户档案成功: {retrieved_profile.name}, {retrieved_profile.age}岁")
    else:
        print("✗ 获取用户档案失败")
    
    # 更新用户档案
    retrieved_profile.age = 31
    memory_manager.update_user_profile(retrieved_profile)
    updated_profile = memory_manager.get_user_profile(test_user_id)
    if updated_profile and updated_profile.age == 31:
        print("✓ 更新用户档案成功")
    else:
        print("✗ 更新用户档案失败")
    
    # 2. 测试偏好设置管理
    print("\n2. 测试偏好设置管理")
    
    # 添加偏好设置
    preference1 = Preference(
        user_id=test_user_id,
        category="娱乐",
        key="音乐类型",
        value="流行音乐"
    )
    memory_manager.add_preference(preference1)
    
    preference2 = Preference(
        user_id=test_user_id,
        category="饮食",
        key="喜欢的食物",
        value="川菜"
    )
    memory_manager.add_preference(preference2)
    print("✓ 添加偏好设置成功")
    
    # 获取偏好设置
    preferences = memory_manager.get_preferences(test_user_id)
    if preferences:
        print(f"✓ 获取偏好设置成功，共 {len(preferences)} 项")
        for pref in preferences:
            print(f"  - {pref.category}: {pref.key} = {pref.value}")
    else:
        print("✗ 获取偏好设置失败")
    
    # 获取特定类别偏好设置
    music_preferences = memory_manager.get_preferences(test_user_id, category="娱乐")
    if music_preferences:
        print("✓ 获取特定类别偏好设置成功")
    else:
        print("✗ 获取特定类别偏好设置失败")
    
    # 更新偏好设置
    preference1.value = "古典音乐"
    memory_manager.update_preference(preference1)
    updated_preferences = memory_manager.get_preferences(test_user_id, category="娱乐")
    if updated_preferences and updated_preferences[0].value == "古典音乐":
        print("✓ 更新偏好设置成功")
    else:
        print("✗ 更新偏好设置失败")
    
    # 3. 测试日历事件管理
    print("\n3. 测试日历事件管理")
    
    # 创建日历事件
    event_id = str(uuid.uuid4())
    event = CalendarEvent(
        event_id=event_id,
        user_id=test_user_id,
        title="测试会议",
        start_time=datetime.now() + timedelta(days=1),
        end_time=datetime.now() + timedelta(days=1, hours=1),
        reminder={"minutes": 30, "method": "notification"},
        description="这是一个测试会议"
    )
    memory_manager.add_calendar_event(event)
    print("✓ 创建日历事件成功")
    
    # 获取日历事件
    events = memory_manager.get_calendar_events(test_user_id)
    if events:
        print(f"✓ 获取日历事件成功，共 {len(events)} 项")
        for e in events:
            print(f"  - {e.title}: {e.start_time}")
    else:
        print("✗ 获取日历事件失败")
    
    # 更新日历事件
    event.title = "更新后的测试会议"
    memory_manager.update_calendar_event(event)
    updated_events = memory_manager.get_calendar_events(test_user_id)
    if updated_events and updated_events[0].title == "更新后的测试会议":
        print("✓ 更新日历事件成功")
    else:
        print("✗ 更新日历事件失败")
    
    # 4. 测试记忆查询
    print("\n4. 测试记忆查询")
    
    # 查询记忆
    query_result = memory_manager.query_memory(test_user_id, "测试")
    if query_result:
        print(f"✓ 查询记忆成功")
        print(f"  - 档案: {len(query_result['profiles'])} 项")
        print(f"  - 偏好: {len(query_result['preferences'])} 项")
        print(f"  - 事件: {len(query_result['events'])} 项")
    else:
        print("✗ 查询记忆失败")
    
    # 5. 测试提醒功能
    print("\n5. 测试提醒功能")
    
    # 创建即将到来的事件
    upcoming_event_id = str(uuid.uuid4())
    upcoming_event = CalendarEvent(
        event_id=upcoming_event_id,
        user_id=test_user_id,
        title="即将到来的测试事件",
        start_time=datetime.now() + timedelta(hours=1),
        reminder={"minutes": 30, "method": "notification"}
    )
    memory_manager.add_calendar_event(upcoming_event)
    
    # 获取即将到来的提醒
    reminders = memory_manager.get_upcoming_reminders(test_user_id, hours=24)
    if reminders:
        print(f"✓ 获取即将到来的提醒成功，共 {len(reminders)} 项")
        for reminder in reminders:
            print(f"  - {reminder['title']}")
    else:
        print("✗ 获取即将到来的提醒失败")
    
    # 6. 测试导入/导出功能
    print("\n6. 测试导入/导出功能")
    
    # 导出记忆
    export_file = memory_manager.export_memory(test_user_id)
    if export_file:
        print(f"✓ 导出记忆成功: {export_file}")
    else:
        print("✗ 导出记忆失败")
    
    # 7. 测试删除功能
    print("\n7. 测试删除功能")
    
    # 删除日历事件
    memory_manager.delete_calendar_event(event_id)
    remaining_events = memory_manager.get_calendar_events(test_user_id)
    if len(remaining_events) == 1:  # 应该还剩一个即将到来的事件
        print("✓ 删除日历事件成功")
    else:
        print("✗ 删除日历事件失败")
    
    # 删除偏好设置
    memory_manager.delete_preference(test_user_id, "娱乐", "音乐类型")
    remaining_preferences = memory_manager.get_preferences(test_user_id)
    if len(remaining_preferences) == 1:  # 应该还剩一个饮食偏好
        print("✓ 删除偏好设置成功")
    else:
        print("✗ 删除偏好设置失败")
    
    # 删除用户档案
    memory_manager.delete_user_profile(test_user_id)
    deleted_profile = memory_manager.get_user_profile(test_user_id)
    if not deleted_profile:
        print("✓ 删除用户档案成功")
    else:
        print("✗ 删除用户档案失败")
    
    print("\n=== 测试完成 ===")

if __name__ == "__main__":
    test_memory_module()
