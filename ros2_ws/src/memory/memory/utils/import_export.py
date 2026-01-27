#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
导入导出工具模块

提供记忆库的导入导出功能
"""

import json
import zipfile
import os
from datetime import datetime

class ImportExport:
    """导入导出工具类"""
    
    def export_memory(self, memory_manager, user_id, export_path=None, format='json'):
        """
        导出记忆库
        
        Args:
            memory_manager (MemoryManager): 记忆库管理器
            user_id (str): 用户ID
            export_path (str, optional): 导出路径
            format (str, optional): 导出格式，支持json和zip
            
        Returns:
            str: 导出文件路径
        """
        if not export_path:
            # 默认导出路径
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            export_path = f'memory_export_{user_id}_{timestamp}'
        
        # 获取用户数据
        data = {
            'user_id': user_id,
            'export_time': datetime.now().isoformat(),
            'data': {
                'profile': None,
                'preferences': [],
                'calendar_events': []
            }
        }
        
        # 获取用户档案
        profile = memory_manager.get_user_profile(user_id)
        if profile:
            data['data']['profile'] = profile.to_dict()
        
        # 获取用户偏好
        preferences = memory_manager.get_preferences(user_id)
        data['data']['preferences'] = [p.to_dict() for p in preferences]
        
        # 获取日历事件
        events = memory_manager.get_calendar_events(user_id)
        data['data']['calendar_events'] = [e.to_dict() for e in events]
        
        if format == 'json':
            # 导出为JSON文件
            export_file = f'{export_path}.json'
            with open(export_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
        elif format == 'zip':
            # 导出为ZIP文件
            export_file = f'{export_path}.zip'
            with zipfile.ZipFile(export_file, 'w') as zipf:
                # 添加JSON数据
                json_data = json.dumps(data, ensure_ascii=False, indent=2)
                zipf.writestr('memory_data.json', json_data)
                
                # 可以添加其他相关文件
                # 例如：用户头像、语音记录等
        
        else:
            raise ValueError(f'不支持的导出格式: {format}')
        
        return export_file
    
    def import_memory(self, memory_manager, import_file, user_id=None):
        """
        导入记忆库
        
        Args:
            memory_manager (MemoryManager): 记忆库管理器
            import_file (str): 导入文件路径
            user_id (str, optional): 目标用户ID，若不提供则使用导出文件中的用户ID
            
        Returns:
            dict: 导入结果
        """
        data = None
        
        # 检查文件类型
        if import_file.endswith('.json'):
            # 读取JSON文件
            with open(import_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
        
        elif import_file.endswith('.zip'):
            # 读取ZIP文件
            with zipfile.ZipFile(import_file, 'r') as zipf:
                # 查找JSON数据文件
                for name in zipf.namelist():
                    if name == 'memory_data.json':
                        with zipf.open(name) as f:
                            data = json.load(f)
                        break
        
        else:
            raise ValueError(f'不支持的导入文件格式: {import_file}')
        
        if not data:
            raise ValueError('无法读取导入文件')
        
        # 确定目标用户ID
        target_user_id = user_id or data.get('user_id')
        if not target_user_id:
            raise ValueError('无法确定目标用户ID')
        
        # 导入用户档案
        profile_data = data['data'].get('profile')
        if profile_data:
            from memory.models.user_profile import UserProfile
            profile = UserProfile.from_dict(profile_data)
            # 更新用户ID
            profile.user_id = target_user_id
            memory_manager.create_user_profile(profile)
        
        # 导入用户偏好
        preferences_data = data['data'].get('preferences', [])
        for pref_data in preferences_data:
            from memory.models.preference import Preference
            preference = Preference.from_dict(pref_data)
            # 更新用户ID
            preference.user_id = target_user_id
            memory_manager.add_preference(preference)
        
        # 导入日历事件
        events_data = data['data'].get('calendar_events', [])
        for event_data in events_data:
            from memory.models.calendar import CalendarEvent
            event = CalendarEvent.from_dict(event_data)
            # 更新用户ID
            event.user_id = target_user_id
            memory_manager.add_calendar_event(event)
        
        return {
            'user_id': target_user_id,
            'import_time': datetime.now().isoformat(),
            'stats': {
                'profile_imported': 1 if profile_data else 0,
                'preferences_imported': len(preferences_data),
                'events_imported': len(events_data)
            }
        }
