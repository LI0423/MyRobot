FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

RUN pip install --no-cache-dir mcp paho-mqtt

COPY scripts/alarm_mcp_server.py /app/scripts/alarm_mcp_server.py

CMD ["python", "/app/scripts/alarm_mcp_server.py"]
