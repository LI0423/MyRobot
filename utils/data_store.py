#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
数据存储工具
"""

import sqlite3
import os
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class DataStore:
    """数据存储类"""
    
    def __init__(self, db_path=None):
        # 支持多种路径方式，确保在ROS2环境中也能找到数据库文件
        if db_path is None:
            # 尝试多种可能的数据库文件路径
            possible_paths = [
                "data/robot.db",  # 原始路径
                "/data/robot.db",  # 绝对路径
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "../data/robot.db"),  # 相对于工具模块的路径
                "ros2_ws/src/data/robot.db"  # ROS2工作空间内的路径
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    db_path = path
                    break
            else:
                db_path = "data/robot.db"  # 默认路径
        
        self.db_path = db_path
        self.conn = None
        self.cursor = None
        
    def connect(self):
        """连接数据库"""
        try:
            # 确保数据目录存在
            db_dir = os.path.dirname(self.db_path)
            if db_dir and not os.path.exists(db_dir):
                os.makedirs(db_dir)
            
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.cursor = self.conn.cursor()
            self._create_tables()
            logger.info(f"数据库连接成功: {self.db_path}")
            return True
            
        except Exception as e:
            logger.error(f"数据库连接失败: {str(e)}")
            return False
    
    def _create_tables(self):
        """创建数据库表"""
        try:
            # 用户表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS users (
                    id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    age INTEGER,
                    gender TEXT,
                    relationship TEXT,
                    face_encoding BLOB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            # 健康数据表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS health_data (
                    id TEXT PRIMARY KEY,
                    user_id TEXT,
                    heart_rate INTEGER,
                    blood_pressure TEXT,
                    blood_glucose REAL,
                    weight REAL,
                    spo2 INTEGER,
                    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (user_id) REFERENCES users(id)
                )
            ''')
            
            # 日程表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS schedules (
                    id TEXT PRIMARY KEY,
                    user_id TEXT,
                    title TEXT NOT NULL,
                    description TEXT,
                    schedule_time TIMESTAMP NOT NULL,
                    repeat TEXT DEFAULT 'none',
                    remind_before INTEGER DEFAULT 5,
                    status TEXT DEFAULT 'active',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (user_id) REFERENCES users(id)
                )
            ''')
            
            # 用药提醒表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS medications (
                    id TEXT PRIMARY KEY,
                    user_id TEXT,
                    name TEXT NOT NULL,
                    dosage TEXT NOT NULL,
                    frequency TEXT DEFAULT 'daily',
                    times TEXT,
                    start_date TIMESTAMP NOT NULL,
                    end_date TIMESTAMP,
                    status TEXT DEFAULT 'active',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (user_id) REFERENCES users(id)
                )
            ''')
            
            # 报警记录表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS alarms (
                    id TEXT PRIMARY KEY,
                    type TEXT NOT NULL,
                    details TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    status TEXT DEFAULT 'unhandled'
                )
            ''')
            
            # 录像记录表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS recordings (
                    id TEXT PRIMARY KEY,
                    start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    end_time TIMESTAMP,
                    duration INTEGER,
                    file_path TEXT,
                    reason TEXT,
                    status TEXT DEFAULT 'completed'
                )
            ''')
            
            # 场景模式表
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS scenes (
                    id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    description TEXT,
                    devices TEXT,
                    status TEXT DEFAULT 'inactive',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            self.conn.commit()
            logger.info("数据库表创建成功")
            
        except Exception as e:
            logger.error(f"创建数据库表失败: {str(e)}")
            self.conn.rollback()
    
    def execute(self, sql, params=None):
        """执行SQL语句
        
        Args:
            sql: SQL语句
            params: SQL参数
            
        Returns:
            tuple: (success, result)
        """
        try:
            if not self.conn or not self.cursor:
                self.connect()
            
            if params:
                self.cursor.execute(sql, params)
            else:
                self.cursor.execute(sql)
            
            self.conn.commit()
            
            # 如果是查询语句，返回查询结果
            if sql.strip().upper().startswith('SELECT'):
                return True, self.cursor.fetchall()
            else:
                return True, self.cursor.rowcount
                
        except Exception as e:
            logger.error(f"执行SQL语句失败: {str(e)}")
            self.conn.rollback()
            return False, None
    
    def insert(self, table, data):
        """插入数据
        
        Args:
            table: 表名
            data: 字典形式的数据
            
        Returns:
            bool: 插入成功返回True，否则返回False
        """
        try:
            columns = ', '.join(data.keys())
            placeholders = ', '.join(['?' for _ in data.values()])
            sql = f"INSERT INTO {table} ({columns}) VALUES ({placeholders})"
            
            success, result = self.execute(sql, tuple(data.values()))
            return success
            
        except Exception as e:
            logger.error(f"插入数据失败: {str(e)}")
            return False
    
    def update(self, table, data, condition):
        """更新数据
        
        Args:
            table: 表名
            data: 字典形式的数据
            condition: 更新条件
            
        Returns:
            bool: 更新成功返回True，否则返回False
        """
        try:
            set_clause = ', '.join([f"{key} = ?" for key in data.keys()])
            sql = f"UPDATE {table} SET {set_clause} WHERE {condition}"
            
            success, result = self.execute(sql, tuple(data.values()))
            return success
            
        except Exception as e:
            logger.error(f"更新数据失败: {str(e)}")
            return False
    
    def delete(self, table, condition):
        """删除数据
        
        Args:
            table: 表名
            condition: 删除条件
            
        Returns:
            bool: 删除成功返回True，否则返回False
        """
        try:
            sql = f"DELETE FROM {table} WHERE {condition}"
            
            success, result = self.execute(sql)
            return success
            
        except Exception as e:
            logger.error(f"删除数据失败: {str(e)}")
            return False
    
    def select(self, table, columns='*', condition=None):
        """查询数据
        
        Args:
            table: 表名
            columns: 查询的列，默认为全部列
            condition: 查询条件
            
        Returns:
            list: 查询结果列表
        """
        try:
            sql = f"SELECT {columns} FROM {table}"
            if condition:
                sql += f" WHERE {condition}"
            
            success, result = self.execute(sql)
            if success:
                return result
            else:
                return []
                
        except Exception as e:
            logger.error(f"查询数据失败: {str(e)}")
            return []
    
    def close(self):
        """关闭数据库连接"""
        try:
            if self.cursor:
                self.cursor.close()
            if self.conn:
                self.conn.close()
            logger.info("数据库连接已关闭")
            return True
            
        except Exception as e:
            logger.error(f"关闭数据库连接失败: {str(e)}")
            return False

# 创建全局数据存储实例
data_store = DataStore()
