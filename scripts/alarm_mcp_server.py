#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
云端 MCP Server（闹钟工具）

依赖:
- mcp[cli] 或 mcp（FastMCP）
- paho-mqtt
"""

import json
import os
import threading
import time
import uuid
from datetime import datetime
from typing import Any, Dict

import paho.mqtt.client as mqtt

try:
    from mcp.server.fastmcp import FastMCP
except ImportError as e:
    raise RuntimeError("请先安装 MCP SDK: pip install mcp") from e


class MQTTToolGateway:
    def __init__(self):
        self.host = os.getenv("MCP_MQTT_HOST", "127.0.0.1")
        self.port = int(os.getenv("MCP_MQTT_PORT", "1883"))
        self.username = os.getenv("MCP_MQTT_USERNAME", "")
        self.password = os.getenv("MCP_MQTT_PASSWORD", "")
        self.call_topic_tpl = os.getenv("MCP_CALL_TOPIC_TEMPLATE", "robot/{device_id}/tool_call")
        self.result_topic_tpl = os.getenv("MCP_RESULT_TOPIC_TEMPLATE", "robot/{device_id}/tool_result")
        self.heartbeat_topic_tpl = os.getenv("MCP_HEARTBEAT_TOPIC_TEMPLATE", "robot/{device_id}/heartbeat")
        self.timeout_sec = float(os.getenv("MCP_TOOL_TIMEOUT_SEC", "15"))
        self.offline_timeout_sec = float(os.getenv("MCP_DEVICE_OFFLINE_TIMEOUT_SEC", "45"))

        self._lock = threading.Lock()
        self._pending: Dict[str, Dict[str, Any]] = {}
        self._device_heartbeats: Dict[str, float] = {}
        self._connected = threading.Event()

        self.client = mqtt.Client(client_id=f"alarm-mcp-server-{uuid.uuid4().hex[:8]}", clean_session=True)
        if self.username:
            self.client.username_pw_set(self.username, self.password)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.client.connect_async(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self._connected.clear()
            return
        self._connected.set()
        # 监听所有端侧回包
        client.subscribe(self.result_topic_tpl.format(device_id="+"), qos=1)
        # 监听所有端侧心跳
        client.subscribe(self.heartbeat_topic_tpl.format(device_id="+"), qos=1)

    def _on_disconnect(self, client, userdata, rc):
        self._connected.clear()

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        device_id = payload.get("device_id")
        if device_id:
            with self._lock:
                self._device_heartbeats[str(device_id)] = time.time()
        request_id = payload.get("request_id")
        if not request_id:
            return

        with self._lock:
            holder = self._pending.get(request_id)
            if not holder:
                return
            holder["result"] = payload
            holder["event"].set()

    def call_tool(self, device_id: str, method: str, params: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(device_id, str) or not device_id.strip():
            return {
                "success": False,
                "error_code": "INVALID_PARAM",
                "error_message": "device_id 必填",
                "result": None,
            }
        if not isinstance(method, str) or not method.strip():
            return {
                "success": False,
                "error_code": "INVALID_PARAM",
                "error_message": "method 必填",
                "result": None,
            }
        if not isinstance(params, dict):
            return {
                "success": False,
                "error_code": "INVALID_PARAM",
                "error_message": "params 必须是对象",
                "result": None,
            }
        if not self._connected.wait(timeout=3.0):
            return {
                "success": False,
                "error_code": "MQTT_DISCONNECTED",
                "error_message": "MQTT 未连接",
                "result": None,
            }
        if not self._is_device_online(device_id.strip()):
            return {
                "success": False,
                "error_code": "DEVICE_OFFLINE",
                "error_message": f"device={device_id} 离线或心跳超时",
                "result": None,
            }

        request_id = uuid.uuid4().hex
        event = threading.Event()
        with self._lock:
            self._pending[request_id] = {"event": event, "result": None}
            self._cleanup_pending_locked()

        request = {
            "request_id": request_id,
            "device_id": device_id,
            "method": method,
            "params": params,
            "timestamp": int(time.time()),
        }
        call_topic = self.call_topic_tpl.format(device_id=device_id.strip())
        info = self.client.publish(call_topic, json.dumps(request, ensure_ascii=False), qos=1)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            with self._lock:
                self._pending.pop(request_id, None)
            return {
                "success": False,
                "error_code": "PUBLISH_FAILED",
                "error_message": f"MQTT publish rc={info.rc}",
                "result": None,
            }

        ok = event.wait(self.timeout_sec)
        with self._lock:
            holder = self._pending.pop(request_id, None)
        if not ok or not holder or holder.get("result") is None:
            return {
                "success": False,
                "error_code": "TIMEOUT",
                "error_message": f"device={device_id} 工具调用超时",
                "result": None,
            }
        return holder["result"]

    def _is_device_online(self, device_id: str) -> bool:
        with self._lock:
            last = self._device_heartbeats.get(device_id)
        if last is None:
            return False
        return (time.time() - last) <= self.offline_timeout_sec

    def _cleanup_pending_locked(self):
        """清理异常残留的 pending 请求"""
        max_pending = 10000
        if len(self._pending) <= max_pending:
            return
        # 极端情况下简单裁剪，避免内存持续增长
        keys = list(self._pending.keys())[: len(self._pending) - max_pending]
        for k in keys:
            self._pending.pop(k, None)


mcp = FastMCP("alarm-mcp-server")
gateway = MQTTToolGateway()


@mcp.tool()
def set_alarm(
    device_id: str,
    due_time: str,
    command: str,
    repeat_rule: str = "",
    source: str = "mcp",
    idempotency_key: str = "",
) -> dict:
    """
    远程设置端侧闹钟
    """
    if not isinstance(due_time, str) or not due_time.strip():
        return {"success": False, "error_code": "INVALID_PARAM", "error_message": "due_time 必填", "result": None}
    if not isinstance(command, str) or not command.strip():
        return {"success": False, "error_code": "INVALID_PARAM", "error_message": "command 必填", "result": None}
    if repeat_rule and repeat_rule not in ("daily", "weekly", "monthly"):
        return {"success": False, "error_code": "INVALID_PARAM", "error_message": "repeat_rule 非法", "result": None}
    try:
        datetime.fromisoformat(due_time)
    except Exception:
        return {"success": False, "error_code": "INVALID_PARAM", "error_message": "due_time 必须是 ISO 时间", "result": None}

    params = {
        "due_time": due_time,
        "command": command,
        "source": source,
    }
    if repeat_rule:
        params["repeat_rule"] = repeat_rule
    if idempotency_key:
        params["idempotency_key"] = idempotency_key

    result = gateway.call_tool(device_id=device_id, method="set_alarm", params=params)
    return result


if __name__ == "__main__":
    mcp.run()
