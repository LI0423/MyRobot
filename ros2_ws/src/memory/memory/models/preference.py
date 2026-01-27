#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
偏好设置模型

存储用户的兴趣爱好、行为习惯等
"""

from datetime import datetime

class Preference:
    """偏好设置类"""
    
    def __init__(self, user_id, category, key, value):
        """
        初始化偏好设置
        
        Args:
            user_id (str): 用户唯一标识
            category (str): 偏好类别
            key (str): 偏好键
            value (any): 偏好值
        """
        self.user_id = user_id
        self.category = category  # 如：娱乐、饮食、生活习惯等
        self.key = key            # 具体偏好项
        self.value = value        # 偏好值
        self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def to_dict(self):
        """
        转换为字典格式
        
        Returns:
            dict: 偏好设置字典
        """
        return {
            'user_id': self.user_id,
            'category': self.category,
            'key': self.key,
            'value': self.value,
            'created_at': self.created_at.isoformat(),
            'updated_at': self.updated_at.isoformat()
        }
    
    @classmethod
    def from_dict(cls, data):
        """
        从字典创建偏好设置
        
        Args:
            data (dict): 偏好设置字典
            
        Returns:
            Preference: 偏好设置对象
        """
        preference = cls(
            user_id=data['user_id'],
            category=data['category'],
            key=data['key'],
            value=data['value']
        )
        if 'created_at' in data:
            preference.created_at = datetime.fromisoformat(data['created_at'])
        if 'updated_at' in data:
            preference.updated_at = datetime.fromisoformat(data['updated_at'])
        return preference
    
    def update(self, value):
        """
        更新偏好值
        
        Args:
            value (any): 新的偏好值
        """
        self.value = value
        self.updated_at = datetime.now()
    
    def __str__(self):
        """
        字符串表示
        
        Returns:
            str: 偏好设置字符串
        """
        return f"Preference(user_id={self.user_id}, category={self.category}, key={self.key}, value={self.value})"
