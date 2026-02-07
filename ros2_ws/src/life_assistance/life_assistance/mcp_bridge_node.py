#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
import time
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
        self.heartbeat_topic = os.getenv("MCP_HEARTBEAT_TOPIC", f"robot/{self.device_id}/heartbeat")
        self.service_timeout_sec = float(os.getenv("MCP_BRIDGE_SERVICE_TIMEOUT_SEC", "15"))
        self.request_ttl_sec = int(os.getenv("MCP_BRIDGE_REQUEST_TTL_SEC", "120"))
        self.heartbeat_interval_sec = float(os.getenv("MCP_BRIDGE_HEARTBEAT_INTERVAL_SEC", "15"))

        self.tool_client = self.create_client(ToolCall, "life_assistance/tool_call")
        self._wait_for_service()

        self._seen_lock = threading.Lock()
        self._seen_requests: Dict[str, float] = {}
        self._stop_event = threading.Event()

        self.mqtt_client = mqtt.Client(client_id=f"mcp-bridge-{self.device_id}", clean_session=True)
        if self.username:
            self.mqtt_client.username_pw_set(self.username, self.password)
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.on_disconnect = self._on_disconnect

        self.get_logger().info(
            f"MCP Bridge 启动: broker={self.broker_host}:{self.broker_port}, call_topic={self.call_topic}"
        )
        self.mqtt_client.connect_async(self.broker_host, self.broker_port, keepalive=30)
        self.mqtt_client.loop_start()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

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
        if self._stop_event.is_set():
            return
        self.get_logger().warning(f"MQTT 断开连接, rc={rc}，等待自动重连")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8")
            request = json.loads(payload)
            if not isinstance(request, dict):
                raise ValueError("请求必须是 JSON 对象")
        except Exception as e:
            self.get_logger().error(f"解析 MQTT 消息失败: {e}")
            return

        request_id = str(request.get("request_id", "")).strip()
        if not request_id:
            self._publish_result(
                request_id="",
                success=False,
                error_code="INVALID_REQUEST",
                error_message="request_id 不能为空",
                result=None,
            )
            return
        if self._is_duplicate_request(request_id):
            self.get_logger().warning(f"忽略重复请求: {request_id}")
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
        done_event = threading.Event()
        holder = {"resp": None, "exc": None}

        def _on_done(fut):
            try:
                holder["resp"] = fut.result()
            except Exception as e:
                holder["exc"] = e
            finally:
                done_event.set()

        future.add_done_callback(_on_done)
        if not done_event.wait(self.service_timeout_sec):
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="TIMEOUT",
                error_message="调用端侧工具超时",
                result=None,
            )
            return
        if holder["exc"] is not None:
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="SERVICE_ERROR",
                error_message=str(holder["exc"]),
                result=None,
            )
            return
        resp = holder["resp"]
        if resp is None:
            self._publish_result(
                request_id=request_id,
                success=False,
                error_code="SERVICE_ERROR",
                error_message="调用端侧工具失败",
                result=None,
            )
            return

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
        info = self.mqtt_client.publish(self.result_topic, json.dumps(payload, ensure_ascii=False), qos=1)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"MQTT 发布失败: rc={info.rc}")

    def _heartbeat_loop(self):
        """周期性上报桥接和工具服务可用性"""
        while not self._stop_event.is_set():
            self._publish_heartbeat()
            self._stop_event.wait(self.heartbeat_interval_sec)

    def _publish_heartbeat(self):
        service_ready = self.tool_client.wait_for_service(timeout_sec=0.1)
        payload = {
            "device_id": self.device_id,
            "ts": int(time.time()),
            "bridge_status": "ok",
            "service_ready": bool(service_ready),
            "pending_seen_cache": len(self._seen_requests),
        }
        info = self.mqtt_client.publish(self.heartbeat_topic, json.dumps(payload, ensure_ascii=False), qos=1)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().warning(f"心跳发布失败: rc={info.rc}")

    def _is_duplicate_request(self, request_id: str) -> bool:
        """QoS1 下消息可能重复投递，按 request_id 去重"""
        now = time.time()
        with self._seen_lock:
            # 清理过期缓存
            expired = [k for k, ts in self._seen_requests.items() if now - ts > self.request_ttl_sec]
            for key in expired:
                self._seen_requests.pop(key, None)
            if request_id in self._seen_requests:
                return True
            self._seen_requests[request_id] = now
            return False

    def destroy_node(self):
        try:
            self._stop_event.set()
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
