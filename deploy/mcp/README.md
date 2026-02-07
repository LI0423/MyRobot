# Alarm MCP Docker Deployment

## 1. Prepare

```bash
cd /Users/litengjiang/Desktop/MyCode/MyRobot/deploy/mcp
cp .env.example .env
```

## 2. Start

```bash
docker compose up -d --build
```

## 3. Check

```bash
docker compose ps
docker compose logs -f alarm-mcp-server
docker compose logs -f mosquitto
```

## 4. Stop

```bash
docker compose down
```

## Notes

- `alarm-mcp-server` uses MQTT topics:
  - call: `robot/{device_id}/tool_call`
  - result: `robot/{device_id}/tool_result`
  - heartbeat: `robot/{device_id}/heartbeat`
- Server side will reject requests when heartbeat is stale:
  - `MCP_DEVICE_OFFLINE_TIMEOUT_SEC` (default: 45s)
- This compose file enables anonymous MQTT access by default for quick local tests.
- For production, disable anonymous access and enable password/TLS in `mosquitto.conf`.
