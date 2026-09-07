ARG MOWGLI_IMAGE=ghcr.io/mowglinext/mowglinext/mowgli-ros2:1.1.0
FROM ${MOWGLI_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pygame \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /bagheera_ws
COPY src/bagheera_base src/bagheera_base

RUN . /opt/ros/kilted/setup.sh \
 && . /ros2_ws/install/setup.sh \
 && colcon build \
      --merge-install \
      --packages-select bagheera_base \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /bagheera_entrypoint.sh
RUN chmod +x /bagheera_entrypoint.sh

ENTRYPOINT ["/bagheera_entrypoint.sh"]
CMD ["ros2", "launch", "bagheera_base", "manual_control.launch.py"]
