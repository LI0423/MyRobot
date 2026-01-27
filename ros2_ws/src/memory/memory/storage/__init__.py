#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
存储后端模块

提供本地存储和云存储实现
"""

# 使用相对导入
from .local_storage import LocalStorage
from .cloud_storage import CloudStorage

__all__ = [
    'LocalStorage',
    'CloudStorage'
]
