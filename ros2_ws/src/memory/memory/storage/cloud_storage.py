#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
云存储实现

提供云存储功能，需用户明确授权
"""

import requests
import json
from datetime import datetime

class CloudStorage:
    """云存储类"""
    
    def __init__(self, api_url, api_key):
        """
        初始化云存储
        
        Args:
            api_url (str): 云存储API URL
            api_key (str): API密钥
        """
        self.api_url = api_url
        self.api_key = api_key
        self.headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }
    
    def _make_request(self, endpoint, method='GET', data=None):
        """
        发送API请求
        
        Args:
            endpoint (str): API端点
            method (str, optional): 请求方法
            data (dict, optional): 请求数据
            
        Returns:
            dict: 响应数据
        """
        url = f'{self.api_url}/{endpoint}'
        
        try:
            if method == 'GET':
                response = requests.get(url, headers=self.headers)
            elif method == 'POST':
                response = requests.post(url, headers=self.headers, json=data)
            elif method == 'PUT':
                response = requests.put(url, headers=self.headers, json=data)
            elif method == 'DELETE':
                response = requests.delete(url, headers=self.headers)
            else:
                raise ValueError(f'不支持的请求方法: {method}')
            
            response.raise_for_status()
            return response.json()
        except Exception as e:
            print(f'云存储请求失败: {str(e)}')
            # 云存储失败不应影响本地功能
            return None
    
    # 用户档案相关方法
    def save_user_profile(self, profile):
        """
        保存用户档案到云存储
        
        Args:
            profile (UserProfile): 用户档案对象
        """
        data = profile.to_dict()
        endpoint = f'user_profiles/{profile.user_id}'
        return self._make_request(endpoint, 'POST', data)
    
    def get_user_profile(self, user_id):
        """
        从云存储获取用户档案
        
        Args:
            user_id (str): 用户ID
            
        Returns:
            dict or None: 用户档案数据
        """
        endpoint = f'user_profiles/{user_id}'
        return self._make_request(endpoint)
    
    def delete_user_profile(self, user_id):
        """
        从云存储删除用户档案
        
        Args:
            user_id (str): 用户ID
        """
        endpoint = f'user_profiles/{user_id}'
        return self._make_request(endpoint, 'DELETE')
    
    # 偏好设置相关方法
    def save_preference(self, preference):
        """
        保存偏好设置到云存储
        
        Args:
            preference (Preference): 偏好设置对象
        """
        data = preference.to_dict()
        endpoint = f'preferences'
        return self._make_request(endpoint, 'POST', data)
    
    def get_preferences(self, user_id, category=None):
        """
        从云存储获取偏好设置
        
        Args:
            user_id (str): 用户ID
            category (str, optional): 偏好类别
            
        Returns:
            list: 偏好设置数据列表
        """
        endpoint = f'preferences/{user_id}'
        if category:
            endpoint += f'?category={category}'
        return self._make_request(endpoint)
    
    def delete_preference(self, user_id, category, key):
        """
        从云存储删除偏好设置
        
        Args:
            user_id (str): 用户ID
            category (str): 偏好类别
            key (str): 偏好键
        """
        endpoint = f'preferences/{user_id}/{category}/{key}'
        return self._make_request(endpoint, 'DELETE')
    
    # 日历事件相关方法
    def save_calendar_event(self, event):
        """
        保存日历事件到云存储
        
        Args:
            event (CalendarEvent): 日历事件对象
        """
        data = event.to_dict()
        endpoint = f'calendar_events/{event.event_id}'
        return self._make_request(endpoint, 'POST', data)
    
    def get_calendar_events(self, user_id, start_time=None, end_time=None):
        """
        从云存储获取日历事件
        
        Args:
            user_id (str): 用户ID
            start_time (datetime, optional): 开始时间
            end_time (datetime, optional): 结束时间
            
        Returns:
            list: 日历事件数据列表
        """
        endpoint = f'calendar_events/{user_id}'
        params = []
        if start_time:
            params.append(f'start_time={start_time.isoformat()}')
        if end_time:
            params.append(f'end_time={end_time.isoformat()}')
        if params:
            endpoint += '?' + '&'.join(params)
        return self._make_request(endpoint)
    
    def delete_calendar_event(self, event_id):
        """
        从云存储删除日历事件
        
        Args:
            event_id (str): 事件ID
        """
        endpoint = f'calendar_events/{event_id}'
        return self._make_request(endpoint, 'DELETE')
    
    # 同步方法
    def sync_all(self, local_storage, user_id):
        """
        同步所有数据到云存储
        
        Args:
            local_storage (LocalStorage): 本地存储对象
            user_id (str): 用户ID
        """
        # 同步用户档案
        profile = local_storage.get_user_profile(user_id)
        if profile:
            self.save_user_profile(profile)
        
        # 同步偏好设置
        preferences = local_storage.get_preferences(user_id)
        for pref in preferences:
            self.save_preference(pref)
        
        # 同步日历事件
        events = local_storage.get_calendar_events(user_id)
        for event in events:
            self.save_calendar_event(event)
        
        return {'status': 'sync_completed'}
    
    def pull_all(self, local_storage, user_id):
        """
        从云存储拉取所有数据
        
        Args:
            local_storage (LocalStorage): 本地存储对象
            user_id (str): 用户ID
        """
        # 拉取用户档案
        profile = self.get_user_profile(user_id)
        if profile:
            local_storage.save_user_profile(profile)
        
        # 拉取偏好设置
        preferences = self.get_preferences(user_id)
        if preferences:
            for pref in preferences:
                local_storage.save_preference(pref)
        
        # 拉取日历事件
        events = self.get_calendar_events(user_id)
        if events:
            for event in events:
                local_storage.save_calendar_event(event)
        
        return {'status': 'pull_completed'}
