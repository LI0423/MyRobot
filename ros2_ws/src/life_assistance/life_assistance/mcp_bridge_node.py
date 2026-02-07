#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
from typing import Any, Dict

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node

from voice_msgs.srv import ToolCall


class MCPBridgeNode(Node):
    """
    端侧 MCP Bridge:
    - 订阅云端下发的 tool_call 消息
    - 调用本地 ROS2 service: /life_assistance/tool_call
    - 把结果发布回云端
    """

    def __init__(self):
        super().__init__("mcp_bridge")

        self.device_id = os.getenv("MCP_DEVICE_ID", "default")
        self.broker_host = os.getenv("MCP_MQTT_HOST", "127.0.0.1")
        self.broker_port = int(os.getenv("MCP_MQTT_PORT", "1883"))
        self.username = os.getenv("MCP_MQTT_USERNAME", "")
        self.password = os.getenv("MCP_MQTT_PASSWORD", "")
        self.call_topic = os.getenv("MCP_CALL_TOPIC", f"robot/{self.device_id}/tool_call")
        self.result_topic = os.getenv("MCP_RESULT_TOPIC", f"robot/{self.device_id}/tool_result")

        self.tool_client = self.create_client(ToolCall, "life_assistance/tool_call")
        self._wait_for_service()

        self.mqtt_client = mqtt.Client(client_id=f"mcp-bridge-{self.device_id}", clean_session=True)
        if self.username:
            self.mqtt_client.username_pw_set(self.username, self.password)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.on_disconnect = self._on_disconnect

        self.get_logger().info(
            f"MCP Bridge 启动: broker={self.broker_host}:{self.broker_port}, call_topic={self.call_topic}"
        )
        self.mqtt_client.connect(self.broker_host, self.broker_port, keepalive=30)
        self.mqtt_client.loop_start()

    def _wait_for_service(self):
        while not self.tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 life_assistance/tool_call service 可用...")

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT 已连接")
            client.subscribe(self.call_topic, qos=1)
        else:
            self.get_logger().error(f"MQTT 连接失败, rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self.get_logger().warning(f"MQTT 断开连接, rc={rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8")
            request = json.loads(payload)
            if not isinstance(request, dict):
                raise ValueError("请求必须是 JSON 对象")
        except Exception as e:
            self.get_logger().error(f"解析 MQTT 消息失败: {e}")
            return

        # 放到线程里，避免阻塞 MQTT 回调线程
        threading.Thread(target=self._handle_request, args=(request,), daemon=True).start()

    def _handle_request(self, request: Dict[str, Any]):
        request_id = request.get("request_id", "")
        method = request.get("method", "")
        params = request.get("params", {})

        if not method:
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="INVALID_REQUEST",
                error_message="method 不能为空",
                result=None,
            )
            return
        if not isinstance(params, dict):
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="INVALID_REQUEST",
                error_message="params 必须是 JSON 对象",
                result=None,
            )
            return

        req = ToolCall.Request()
        req.method = method
        req.params_json = json.dumps(params, ensure_ascii=False)

        future = self.tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="TIMEOUT",
                error_message="调用端侧工具超时",
                result=None,
            )
            return

        resp = future.result()
        result_payload = None
        if resp.result_json:
            try:
                result_payload = json.loads(resp.result_json)
            except Exception:
                result_payload = {"raw": resp.result_json}

        self._publish_result(
            request_id=request_id,
            success=resp.success,
            error_code=resp.error_code,
            error_message=resp.error_message,
            result=result_payload,
        )

    def _publish_result(self, request_id, success, error_code, error_message, result):
        payload = {
            "request_id": request_id,
            "device_id": self.device_id,
            "success": bool(success),
            "error_code": error_code or "",
            "error_message": error_message or "",
            "result": result,
        }
        self.mqtt_client.publish(self.result_topic, json.dumps(payload, ensure_ascii=False), qos=1)

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MCPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
