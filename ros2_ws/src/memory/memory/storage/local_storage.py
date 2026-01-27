#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本地存储实现

使用SQLite数据库存储用户数据
"""

import sqlite3
import json
import os
from datetime import datetime
from ..utils.encryption import aes_encrypt, aes_decrypt

class LocalStorage:
    """本地存储类"""
    
    def __init__(self, db_path='memory.db'):
        """
        初始化本地存储
        
        Args:
            db_path (str): 数据库文件路径
        """
        self.db_path = db_path
        self._init_db()
    
    def _init_db(self):
        """
        初始化数据库
        """
        # 确保数据库目录存在
        db_dir = os.path.dirname(self.db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir)
        
        # 连接数据库
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # 创建用户档案表
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS user_profiles (
            user_id TEXT PRIMARY KEY,
            data TEXT NOT NULL,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        )
        ''')
        
        # 创建偏好设置表
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS preferences (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id TEXT NOT NULL,
            category TEXT NOT NULL,
            key TEXT NOT NULL,
            value TEXT NOT NULL,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL,
            UNIQUE(user_id, category, key)
        )
        ''')
        
        # 创建日历事件表
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS calendar_events (
            event_id TEXT PRIMARY KEY,
            user_id TEXT NOT NULL,
            data TEXT NOT NULL,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        )
        ''')
        
        # 创建索引
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_preferences_user_id ON preferences(user_id)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_calendar_events_user_id ON calendar_events(user_id)')
        
        # 提交并关闭
        conn.commit()
        conn.close()
    
    def _connect(self):
        """
        连接数据库
        
        Returns:
            tuple: (conn, cursor)
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        return conn, cursor
    
    # 用户档案相关方法
    def save_user_profile(self, profile):
        """
        保存用户档案
        
        Args:
            profile (UserProfile): 用户档案对象
        """
        data = profile.to_dict()
        encrypted_data = aes_encrypt(json.dumps(data))
        
        conn, cursor = self._connect()
        
        try:
            # 尝试更新
            cursor.execute('''
            UPDATE user_profiles 
            SET data = ?, updated_at = ? 
            WHERE user_id = ?
            ''', (encrypted_data, datetime.now().isoformat(), profile.user_id))
            
            # 如果没有更新，则插入
            if cursor.rowcount == 0:
                cursor.execute('''
                INSERT INTO user_profiles (user_id, data, created_at, updated_at)
                VALUES (?, ?, ?, ?)
                ''', (profile.user_id, encrypted_data, data['created_at'], data['updated_at']))
            
            conn.commit()
        finally:
            conn.close()
    
    def get_user_profile(self, user_id):
        """
        获取用户档案
        
        Args:
            user_id (str): 用户ID
            
        Returns:
            dict or None: 用户档案数据
        """
        conn, cursor = self._connect()
        
        try:
            cursor.execute('''
            SELECT data FROM user_profiles WHERE user_id = ?
            ''', (user_id,))
            
            row = cursor.fetchone()
            if row:
                encrypted_data = row[0]
                decrypted_data = aes_decrypt(encrypted_data)
                return json.loads(decrypted_data)
            return None
        finally:
            conn.close()
    
    def delete_user_profile(self, user_id):
        """
        删除用户档案
        
        Args:
            user_id (str): 用户ID
        """
        conn, cursor = self._connect()
        
        try:
            cursor.execute('DELETE FROM user_profiles WHERE user_id = ?', (user_id,))
            conn.commit()
        finally:
            conn.close()
    
    # 偏好设置相关方法
    def save_preference(self, preference):
        """
        保存偏好设置
        
        Args:
            preference (Preference): 偏好设置对象
        """
        data = preference.to_dict()
        
        conn, cursor = self._connect()
        
        try:
            # 尝试更新
            cursor.execute('''
            UPDATE preferences 
            SET value = ?, updated_at = ? 
            WHERE user_id = ? AND category = ? AND key = ?
            ''', (json.dumps(data['value']), data['updated_at'], preference.user_id, preference.category, preference.key))
            
            # 如果没有更新，则插入
            if cursor.rowcount == 0:
                cursor.execute('''
                INSERT INTO preferences (user_id, category, key, value, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?)
                ''', (preference.user_id, preference.category, preference.key, json.dumps(data['value']), data['created_at'], data['updated_at']))
            
            conn.commit()
        finally:
            conn.close()
    
    def get_preferences(self, user_id, category=None):
        """
        获取偏好设置

        Args:
            user_id (str): 用户ID
            category (str, optional): 偏好类别

        Returns:
            list: 偏好设置数据列表
        """
        conn, cursor = self._connect()

        try:
            if category:
                cursor.execute('''
                SELECT * FROM preferences 
                WHERE user_id = ? AND category = ?
                ORDER BY category, key
                ''', (user_id, category))
            else:
                cursor.execute('''
                SELECT * FROM preferences 
                WHERE user_id = ?
                ORDER BY category, key
                ''', (user_id,))

            preferences = []
            for row in cursor.fetchall():
                # 直接从表中获取
                pref_data = {
                    'user_id': row[1],
                    'category': row[2],
                    'key': row[3],
                    'value': json.loads(row[4]),
                    'created_at': row[5],
                    'updated_at': row[6]
                }
                preferences.append(pref_data)

            return preferences
        finally:
            conn.close()
    
    def delete_preference(self, user_id, category, key):
        """
        删除偏好设置
        
        Args:
            user_id (str): 用户ID
            category (str): 偏好类别
            key (str): 偏好键
        """
        conn, cursor = self._connect()
        
        try:
            cursor.execute('''
            DELETE FROM preferences 
            WHERE user_id = ? AND category = ? AND key = ?
            ''', (user_id, category, key))
            conn.commit()
        finally:
            conn.close()
    
    # 日历事件相关方法
    def save_calendar_event(self, event):
        """
        保存日历事件
        
        Args:
            event (CalendarEvent): 日历事件对象
        """
        data = event.to_dict()
        encrypted_data = aes_encrypt(json.dumps(data))
        
        conn, cursor = self._connect()
        
        try:
            # 尝试更新
            cursor.execute('''
            UPDATE calendar_events 
            SET data = ?, updated_at = ? 
            WHERE event_id = ?
            ''', (encrypted_data, data['updated_at'], event.event_id))
            
            # 如果没有更新，则插入
            if cursor.rowcount == 0:
                cursor.execute('''
                INSERT INTO calendar_events (event_id, user_id, data, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?)
                ''', (event.event_id, event.user_id, encrypted_data, data['created_at'], data['updated_at']))
            
            conn.commit()
        finally:
            conn.close()
    
    def get_calendar_events(self, user_id, start_time=None, end_time=None):
        """
        获取日历事件
        
        Args:
            user_id (str): 用户ID
            start_time (datetime, optional): 开始时间
            end_time (datetime, optional): 结束时间
            
        Returns:
            list: 日历事件数据列表
        """
        conn, cursor = self._connect()
        
        try:
            cursor.execute('''
            SELECT data FROM calendar_events 
            WHERE user_id = ?
            ORDER BY created_at
            ''', (user_id,))
            
            events = []
            for row in cursor.fetchall():
                encrypted_data = row[0]
                decrypted_data = aes_decrypt(encrypted_data)
                event_data = json.loads(decrypted_data)
                
                # 过滤时间范围
                if start_time or end_time:
                    event_start = datetime.fromisoformat(event_data['start_time'])
                    if start_time and event_start < start_time:
                        continue
                    if end_time and event_start > end_time:
                        continue
                
                events.append(event_data)
            
            return events
        finally:
            conn.close()
    
    def delete_calendar_event(self, event_id):
        """
        删除日历事件
        
        Args:
            event_id (str): 事件ID
        """
        conn, cursor = self._connect()
        
        try:
            cursor.execute('DELETE FROM calendar_events WHERE event_id = ?', (event_id,))
            conn.commit()
        finally:
            conn.close()
    
    # 其他方法
    def delete_all_data(self, user_id):
        """
        删除用户所有数据
        
        Args:
            user_id (str): 用户ID
        """
        conn, cursor = self._connect()
        
        try:
            # 删除用户档案
            cursor.execute('DELETE FROM user_profiles WHERE user_id = ?', (user_id,))
            
            # 删除偏好设置
            cursor.execute('DELETE FROM preferences WHERE user_id = ?', (user_id,))
            
            # 删除日历事件
            cursor.execute('DELETE FROM calendar_events WHERE user_id = ?', (user_id,))
            
            conn.commit()
        finally:
            conn.close()
