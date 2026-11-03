# The MowBite Planner on a computer in the house (the planner setting remote_planning, see
# ros2/mowbite_planner/mowbite_planner/house.py): plans for the mower, which checks every plan itself and plans on its
# own where this isn't there. Built on the house computer, see compose.yaml. The mower's own stack (OpenMower's image:
# Ubuntu 20.04, its Python 3.8 and numpy, shapely 2.0.7): the mower takes plans only from the same. The planner's code
# isn't in it: watch.py fetches the code the mower runs from the planner's repository and starts the service with it.
FROM ubuntu:focal@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        python3 python3-pip python3-numpy git ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir shapely==2.0.7 paho-mqtt==2.1.0 \
    && useradd --system --create-home mowbite && mkdir /data && chown mowbite /data
COPY house/watch.py /opt/mowbite/watch.py
USER mowbite
ENV PYTHONDONTWRITEBYTECODE=1 HOUSE_DATA=/data
CMD ["python3", "/opt/mowbite/watch.py"]
