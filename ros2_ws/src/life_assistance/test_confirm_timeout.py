#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
生活助手确认/超时逻辑测试
"""

import unittest
from datetime import datetime, timedelta
import os
import sys

import rclpy

# 确保可直接从源码运行测试
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from life_assistance.schedule_reminder_node import ScheduleReminderNode


class ReminderConfirmTimeoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ScheduleReminderNode()
        self.node.timer.cancel()

        # 屏蔽持久化副作用，聚焦状态机逻辑测试
        self.node._persist_reminder = lambda *args, **kwargs: None
        self.node._update_reminder_fields = lambda *args, **kwargs: None

        self.node.alarms = []
        self.node.medication_reminders = []
        self.node.current_time = datetime.now()

    def tearDown(self):
        self.node.destroy_node()

    def test_confirm_reminder_by_id(self):
        alarm_time = self.node.current_time + timedelta(minutes=1)
        self.node.set_alarm(alarm_time, "设置闹钟 1分钟后")
        alarm_id = self.node.alarms[-1]["id"]

        self.node.alarms[-1]["status"] = "triggered"
        self.node.confirm_reminder(alarm_id, source="test")

        self.assertEqual(self.node.alarms[-1]["status"], "confirmed")

    def test_timeout_transition_to_expired(self):
        self.node.config["confirm_timeout_minutes"] = 0
        due_time = self.node.current_time - timedelta(seconds=1)
        self.node.add_medication_reminder(due_time, "降压药")

        self.node.check_alarms()

        self.assertEqual(self.node.medication_reminders[-1]["status"], "expired")

    def test_snooze_latest_to_pending_with_new_due_time(self):
        due_time = self.node.current_time - timedelta(seconds=1)
        self.node.set_alarm(due_time, "设置闹钟 立即触发")

        self.node.check_alarms()
        self.assertEqual(self.node.alarms[-1]["status"], "triggered")

        before = self.node.current_time
        self.node.snooze_latest(minutes=5, source="test")

        self.assertEqual(self.node.alarms[-1]["status"], "pending")
        self.assertFalse(self.node.alarms[-1]["triggered"])
        self.assertGreaterEqual(
            self.node.alarms[-1]["time"],
            before + timedelta(minutes=5),
        )

    def test_cancel_reminder_should_not_be_expired_status(self):
        alarm_time = self.node.current_time + timedelta(minutes=1)
        self.node.set_alarm(alarm_time, "设置闹钟 1分钟后")
        alarm_id = self.node.alarms[-1]["id"]

        captured = {}
        self.node._update_reminder_fields = lambda reminder_id, status=None, payload=None: captured.update(
            {"id": reminder_id, "status": status}
        )
        self.node.cancel_reminder(alarm_id, source="test")

        self.assertEqual(captured.get("id"), alarm_id)
        self.assertEqual(captured.get("status"), "canceled")

    def test_parse_alarm_time_invalid_hour_should_return_none(self):
        invalid = self.node.parse_alarm_time("设置闹钟 25点10分")
        self.assertIsNone(invalid)


if __name__ == "__main__":
    unittest.main()
