#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
记忆库管理器

整合所有存储功能，提供统一的记忆管理接口
"""

import uuid
from datetime import datetime
from .storage.local_storage import LocalStorage
from .storage.cloud_storage import CloudStorage
from .models.user_profile import UserProfile
from .models.preference import Preference
from .models.calendar import CalendarEvent

class MemoryManager:
    """记忆库管理器类"""
    
    def __init__(self, storage_backend='local', db_path='memory.db', cloud_config=None):
        """
        初始化记忆库管理器
        
        Args:
            storage_backend (str, optional): 存储后端，支持'local'或'cloud'
            db_path (str, optional): 本地数据库路径
            cloud_config (dict, optional): 云存储配置
        """
        # 初始化本地存储
        self.local_storage = LocalStorage(db_path)
        
        # 初始化云存储（如果配置）
        self.cloud_storage = None
        if storage_backend == 'cloud' and cloud_config:
            try:
                self.cloud_storage = CloudStorage(
                    api_url=cloud_config.get('api_url'),
                    api_key=cloud_config.get('api_key')
                )
            except Exception as e:
                print(f'云存储初始化失败: {str(e)}')
                # 云存储失败时回退到本地存储
                storage_backend = 'local'
        
        self.storage_backend = storage_backend
        self.cloud_config = cloud_config
    
    # 用户档案相关方法
    def create_user_profile(self, profile):
        """
        创建用户档案
        
        Args:
            profile (UserProfile): 用户档案对象
        """
        # 保存到本地存储
        self.local_storage.save_user_profile(profile)
        
        # 如果配置了云存储，也保存到云存储
        if self.cloud_storage:
            self.cloud_storage.save_user_profile(profile)
    
    def get_user_profile(self, user_id):
        """
        获取用户档案
        
        Args:
            user_id (str): 用户ID
            
        Returns:
            UserProfile or None: 用户档案对象
        """
        # 从本地存储获取
        profile_data = self.local_storage.get_user_profile(user_id)
        
        if profile_data:
            return UserProfile.from_dict(profile_data)
        return None
    
    def update_user_profile(self, profile):
        """
        更新用户档案
        
        Args:
            profile (UserProfile): 用户档案对象
        """
        # 更新本地存储
        self.local_storage.save_user_profile(profile)
        
        # 如果配置了云存储，也更新云存储
        if self.cloud_storage:
            self.cloud_storage.save_user_profile(profile)
    
    def delete_user_profile(self, user_id):
        """
        删除用户档案
        
        Args:
            user_id (str): 用户ID
        """
        # 从本地存储删除
        self.local_storage.delete_user_profile(user_id)
        
        # 如果配置了云存储，也从云存储删除
        if self.cloud_storage:
            self.cloud_storage.delete_user_profile(user_id)
    
    # 偏好设置相关方法
    def add_preference(self, preference):
        """
        添加偏好设置
        
        Args:
            preference (Preference): 偏好设置对象
        """
        # 保存到本地存储
        self.local_storage.save_preference(preference)
        
        # 如果配置了云存储，也保存到云存储
        if self.cloud_storage:
            self.cloud_storage.save_preference(preference)
    
    def get_preferences(self, user_id, category=None):
        """
        获取偏好设置
        
        Args:
            user_id (str): 用户ID
            category (str, optional): 偏好类别
            
        Returns:
            list: 偏好设置对象列表
        """
        # 从本地存储获取
        preferences_data = self.local_storage.get_preferences(user_id, category)
        
        preferences = []
        for pref_data in preferences_data:
            preferences.append(Preference.from_dict(pref_data))
        
        return preferences
    
    def update_preference(self, preference):
        """
        更新偏好设置
        
        Args:
            preference (Preference): 偏好设置对象
        """
        # 更新本地存储
        self.local_storage.save_preference(preference)
        
        # 如果配置了云存储，也更新云存储
        if self.cloud_storage:
            self.cloud_storage.save_preference(preference)
    
    def delete_preference(self, user_id, category, key):
        """
        删除偏好设置
        
        Args:
            user_id (str): 用户ID
            category (str): 偏好类别
            key (str): 偏好键
        """
        # 从本地存储删除
        self.local_storage.delete_preference(user_id, category, key)
        
        # 如果配置了云存储，也从云存储删除
        if self.cloud_storage:
            self.cloud_storage.delete_preference(user_id, category, key)
    
    # 日历事件相关方法
    def add_calendar_event(self, event):
        """
        添加日历事件
        
        Args:
            event (CalendarEvent): 日历事件对象
        """
        # 保存到本地存储
        self.local_storage.save_calendar_event(event)
        
        # 如果配置了云存储，也保存到云存储
        if self.cloud_storage:
            self.cloud_storage.save_calendar_event(event)
    
    def get_calendar_events(self, user_id, start_time=None, end_time=None):
        """
        获取日历事件
        
        Args:
            user_id (str): 用户ID
            start_time (datetime, optional): 开始时间
            end_time (datetime, optional): 结束时间
            
        Returns:
            list: 日历事件对象列表
        """
        # 从本地存储获取
        events_data = self.local_storage.get_calendar_events(user_id, start_time, end_time)
        
        events = []
        for event_data in events_data:
            events.append(CalendarEvent.from_dict(event_data))
        
        return events
    
    def update_calendar_event(self, event):
        """
        更新日历事件
        
        Args:
            event (CalendarEvent): 日历事件对象
        """
        # 更新本地存储
        self.local_storage.save_calendar_event(event)
        
        # 如果配置了云存储，也更新云存储
        if self.cloud_storage:
            self.cloud_storage.save_calendar_event(event)
    
    def delete_calendar_event(self, event_id):
        """
        删除日历事件
        
        Args:
            event_id (str): 事件ID
        """
        # 从本地存储删除
        self.local_storage.delete_calendar_event(event_id)
        
        # 如果配置了云存储，也从云存储删除
        if self.cloud_storage:
            self.cloud_storage.delete_calendar_event(event_id)
    
    # 记忆管理方法
    def query_memory(self, user_id, query, time_range=None):
        """
        查询记忆
        
        Args:
            user_id (str): 用户ID
            query (str): 查询关键词
            time_range (tuple, optional): 时间范围 (start_time, end_time)
            
        Returns:
            dict: 查询结果
        """
        results = {
            'profiles': [],
            'preferences': [],
            'events': []
        }
        
        # 查询用户档案
        profile = self.get_user_profile(user_id)
        if profile:
            # 检查用户档案是否包含查询关键词
            profile_text = f"{profile.name} {profile.age} {profile.gender} {profile.relationship}".lower()
            if query.lower() in profile_text:
                results['profiles'].append(profile.to_dict())
        
        # 查询偏好设置
        preferences = self.get_preferences(user_id)
        for pref in preferences:
            # 检查偏好设置是否包含查询关键词
            pref_text = f"{pref.category} {pref.key} {str(pref.value)}".lower()
            if query.lower() in pref_text:
                results['preferences'].append(pref.to_dict())
        
        # 查询日历事件
        events = self.get_calendar_events(user_id, 
                                        start_time=time_range[0] if time_range else None,
                                        end_time=time_range[1] if time_range else None)
        for event in events:
            # 检查日历事件是否包含查询关键词
            event_text = f"{event.title} {event.description}".lower()
            if query.lower() in event_text:
                results['events'].append(event.to_dict())
        
        return results
    
    def delete_memory(self, memory_type, memory_id):
        """
        删除记忆
        
        Args:
            memory_type (str): 记忆类型，支持 'profile', 'preference', 'event'
            memory_id (str): 记忆ID
        """
        if memory_type == 'profile':
            self.delete_user_profile(memory_id)
        elif memory_type == 'event':
            self.delete_calendar_event(memory_id)
        # 偏好设置需要额外的category和key参数
        # 这里简化处理，实际应用中可能需要更复杂的逻辑
    
    def export_memory(self, user_id, format='json'):
        """
        导出记忆

        Args:
            user_id (str): 用户ID
            format (str, optional): 导出格式

        Returns:
            str: 导出文件路径
        """
        from .utils.import_export import ImportExport

        exporter = ImportExport()
        return exporter.export_memory(self, user_id, format)

    def import_memory(self, import_file, user_id=None):
        """
        导入记忆

        Args:
            import_file (str): 导入文件路径
            user_id (str, optional): 目标用户ID

        Returns:
            dict: 导入结果
        """
        from .utils.import_export import ImportExport

        importer = ImportExport()
        return importer.import_memory(self, import_file, user_id)
    
    # 提醒相关方法
    def get_upcoming_reminders(self, user_id, hours=24):
        """
        获取即将到来的提醒
        
        Args:
            user_id (str): 用户ID
            hours (int, optional): 时间范围（小时）
            
        Returns:
            list: 提醒列表
        """
        reminders = []
        
        # 获取日历事件
        events = self.get_calendar_events(user_id)
        
        for event in events:
            if event.is_upcoming(hours):
                reminders.append(event.to_dict())
        
        return reminders
    
    # 同步方法
    def sync_with_cloud(self, user_id):
        """
        与云存储同步
        
        Args:
            user_id (str): 用户ID
            
        Returns:
            dict: 同步结果
        """
        if not self.cloud_storage:
            return {'status': 'error', 'message': '云存储未配置'}
        
        try:
            result = self.cloud_storage.sync_all(self.local_storage, user_id)
            return result
        except Exception as e:
            return {'status': 'error', 'message': str(e)}
    
    def pull_from_cloud(self, user_id):
        """
        从云存储拉取数据
        
        Args:
            user_id (str): 用户ID
            
        Returns:
            dict: 拉取结果
        """
        if not self.cloud_storage:
            return {'status': 'error', 'message': '云存储未配置'}
        
        try:
            result = self.cloud_storage.pull_all(self.local_storage, user_id)
            return result
        except Exception as e:
            return {'status': 'error', 'message': str(e)}
