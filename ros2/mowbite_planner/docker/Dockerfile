# The MowBite Planner as a JSON-RPC service over MQTT, next to OpenMower (mowbite_planner/mqtt_service.py).
# Python on alpine, shapely and paho-mqtt as wheels (musllinux, amd64 and arm64), only the planner's modules.
#
#   docker build -t mowbite-planner -f docker/Dockerfile .     (in ros2/mowbite_planner)
#   docker run -d --name mowbite-planner --network host --restart unless-stopped mowbite-planner
#
# --network host: the mower's broker is on 127.0.0.1:1883 (OpenMower OS publishes Mosquitto's port on the
# host). Settings come from the environment: MOWER_MQTT_HOST, MOWER_MQTT_PORT, MOWER_MQTT_USER,
# MOWER_MQTT_PASSWORD, MOWER_MQTT_PREFIX, PLANNER_SETTINGS (see docker/compose.yaml).
# The image is about 145 MB, a 39 MB download: python:3.12-alpine 75 MB, numpy (shapely needs it) 53 MB,
# shapely 13 MB. No .pyc for the packages (5 MB more), python compiles them in memory when it starts.

FROM python:3.12-alpine AS deps
RUN pip install --no-cache-dir --no-compile --only-binary=:all: --prefix=/deps 'shapely>=2,<3' 'paho-mqtt>=1.6,<3' \
 && find /deps -depth -type d -name tests -exec rm -rf {} +

FROM python:3.12-alpine
COPY --from=deps /deps /usr/local
COPY mowbite_planner/*.py /app/mowbite_planner/
RUN printf '#!/bin/sh\nexec python3 -m mowbite_planner.mqtt_service "$@"\n' > /usr/local/bin/mowbite_planner_mqtt \
 && chmod 755 /usr/local/bin/mowbite_planner_mqtt
ENV PYTHONPATH=/app PYTHONUNBUFFERED=1
USER nobody
CMD ["mowbite_planner_mqtt"]
