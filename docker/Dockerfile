ARG MOWGLI_IMAGE=ghcr.io/mowglinext/mowglinext/mowgli-ros2:1.1.0
FROM ${MOWGLI_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG YDLIDAR_SDK_REF=01cdda4f2b36dff2a706d0535c64228d863c7411
ARG YDLIDAR_ROS2_REF=4ef70d3f32a85704ade0be54b214f3763b1ab3e8
ARG CAMERA_ROS_REF=8f792e27a6dbc81e4943a75765fc1b7b7d37b301

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-pygame \
      python3-smbus \
      python3-spidev \
      ros-kilted-camera-ros \
      ros-kilted-foxglove-bridge \
      ros-kilted-nav2-bringup \
      ros-kilted-robot-localization \
      ros-kilted-slam-toolbox \
 && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /opt/ydlidar-sdk-src \
 && git -C /opt/ydlidar-sdk-src checkout "${YDLIDAR_SDK_REF}" \
 && cmake -S /opt/ydlidar-sdk-src -B /opt/ydlidar-sdk-src/build \
      -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /opt/ydlidar-sdk-src/build -j2 \
 && cmake --install /opt/ydlidar-sdk-src/build \
 && ldconfig

WORKDIR /bagheera_ws
COPY docker/ydlidar-reliable-scan.patch /tmp/ydlidar-reliable-scan.patch
RUN git clone --branch humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git \
      src/ydlidar_ros2_driver \
 && git -C src/ydlidar_ros2_driver checkout "${YDLIDAR_ROS2_REF}" \
 && git -C src/ydlidar_ros2_driver apply /tmp/ydlidar-reliable-scan.patch \
 && git clone https://github.com/christianrauch/camera_ros.git src/camera_ros \
 && git -C src/camera_ros checkout "${CAMERA_ROS_REF}"

RUN . /opt/ros/kilted/setup.sh \
 && . /ros2_ws/install/setup.sh \
 && colcon build \
      --merge-install \
      --packages-select ydlidar_ros2_driver camera_ros \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY src/bagheera_base src/bagheera_base
RUN . /opt/ros/kilted/setup.sh \
 && . /ros2_ws/install/setup.sh \
 && . /bagheera_ws/install/setup.sh \
 && colcon build \
      --merge-install \
      --packages-select bagheera_base

COPY docker/entrypoint.sh /bagheera_entrypoint.sh
RUN chmod +x /bagheera_entrypoint.sh

ENTRYPOINT ["/bagheera_entrypoint.sh"]
CMD ["ros2", "launch", "bagheera_base", "manual_control.launch.py"]
