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
        self.timeout_sec = float(os.getenv("MCP_TOOL_TIMEOUT_SEC", "15"))

        self._lock = threading.Lock()
        self._pending: Dict[str, Dict[str, Any]] = {}

        self.client = mqtt.Client(client_id=f"alarm-mcp-server-{uuid.uuid4().hex[:8]}", clean_session=True)
        if self.username:
            self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            return
        # 监听所有端侧回包
        client.subscribe(self.result_topic_tpl.format(device_id="+"), qos=1)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
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
        request_id = uuid.uuid4().hex
        event = threading.Event()
        with self._lock:
            self._pending[request_id] = {"event": event, "result": None}

        request = {
            "request_id": request_id,
            "device_id": device_id,
            "method": method,
            "params": params,
            "timestamp": int(time.time()),
        }
        call_topic = self.call_topic_tpl.format(device_id=device_id)
        self.client.publish(call_topic, json.dumps(request, ensure_ascii=False), qos=1)

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
