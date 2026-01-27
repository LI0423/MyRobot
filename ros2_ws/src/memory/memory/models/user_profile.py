#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
用户档案模型

存储用户基本信息、身份特征等
"""

from datetime import datetime
import json

class UserProfile:
    """用户档案类"""
    
    def __init__(self, user_id, name, age=None, gender=None, relationship=None):
        """
        初始化用户档案
        
        Args:
            user_id (str): 用户唯一标识
            name (str): 用户姓名
            age (int, optional): 用户年龄
            gender (str, optional): 用户性别
            relationship (str, optional): 与机器人的关系
        """
        self.user_id = user_id
        self.name = name
        self.age = age
        self.gender = gender
        self.relationship = relationship  # 如：主人、家人、朋友等
        self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def to_dict(self):
        """
        转换为字典格式
        
        Returns:
            dict: 用户档案字典
        """
        return {
            'user_id': self.user_id,
            'name': self.name,
            'age': self.age,
            'gender': self.gender,
            'relationship': self.relationship,
            'created_at': self.created_at.isoformat(),
            'updated_at': self.updated_at.isoformat()
        }
    
    @classmethod
    def from_dict(cls, data):
        """
        从字典创建用户档案
        
        Args:
            data (dict): 用户档案字典
            
        Returns:
            UserProfile: 用户档案对象
        """
        profile = cls(
            user_id=data['user_id'],
            name=data['name'],
            age=data.get('age'),
            gender=data.get('gender'),
            relationship=data.get('relationship')
        )
        if 'created_at' in data:
            profile.created_at = datetime.fromisoformat(data['created_at'])
        if 'updated_at' in data:
            profile.updated_at = datetime.fromisoformat(data['updated_at'])
        return profile
    
    def update(self, **kwargs):
        """
        更新用户档案
        
        Args:
            **kwargs: 要更新的属性
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.updated_at = datetime.now()
    
    def __str__(self):
        """
        字符串表示
        
        Returns:
            str: 用户档案字符串
        """
        return f"UserProfile(user_id={self.user_id}, name={self.name}, age={self.age}, gender={self.gender}, relationship={self.relationship})"
