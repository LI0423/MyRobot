#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
提醒事项模型

用于存储闹钟、用药提醒、日程提醒等
"""

from datetime import datetime


class ReminderRecord:
    """提醒事项"""

    def __init__(
        self,
        reminder_id,
        user_id,
        kind,
        due_time,
        repeat_rule=None,
        status="pending",
        payload=None,
    ):
        self.reminder_id = reminder_id
        self.user_id = user_id
        self.kind = kind
        self.due_time = due_time
        self.repeat_rule = repeat_rule
        self.status = status
        self.payload = payload or {}
        self.created_at = datetime.now()
        self.updated_at = datetime.now()

    def to_dict(self):
        return {
            "reminder_id": self.reminder_id,
            "user_id": self.user_id,
            "kind": self.kind,
            "due_time": self.due_time.isoformat(),
            "repeat_rule": self.repeat_rule,
            "status": self.status,
            "payload": self.payload,
            "created_at": self.created_at.isoformat(),
            "updated_at": self.updated_at.isoformat(),
        }

    @classmethod
    def from_dict(cls, data):
        record = cls(
            reminder_id=data["reminder_id"],
            user_id=data["user_id"],
            kind=data["kind"],
            due_time=datetime.fromisoformat(data["due_time"]),
            repeat_rule=data.get("repeat_rule"),
            status=data.get("status", "pending"),
            payload=data.get("payload") or {},
        )
        if "created_at" in data:
            record.created_at = datetime.fromisoformat(data["created_at"])
        if "updated_at" in data:
            record.updated_at = datetime.fromisoformat(data["updated_at"])
        return record

    def update(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.updated_at = datetime.now()
