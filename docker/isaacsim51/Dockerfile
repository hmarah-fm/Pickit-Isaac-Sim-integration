FROM nvcr.io/nvidia/isaac-sim:5.1.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root

ENV DEBIAN_FRONTEND=noninteractive \
    ACCEPT_EULA=Y \
    OMNI_ENV_PRIVACY_CONSENT=Y \
    NVIDIA_DRIVER_CAPABILITIES=all

# Isaac Sim 5.1.0 is Ubuntu 24.04 (Noble). Install native Jazzy only for the
# external converter process; launch-isaac-jazzy.sh deliberately does not
# source this environment before starting Isaac Sim.
RUN . /etc/os-release && test "${VERSION_CODENAME}" = "noble" && \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common && \
    add-apt-repository -y universe && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      python3-pip \
      python3-colcon-common-extensions \
      ros-jazzy-ros-base \
      ros-jazzy-pcl-ros \
      ros-jazzy-sensor-msgs \
      ros-jazzy-sensor-msgs-py \
      ros-jazzy-rmw-fastrtps-cpp \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-rosidl-default-runtime && \
    rm -rf /var/lib/apt/lists/*

# These packages are used by the converter's native Python 3.12 process.
RUN python3 -m pip install --no-cache-dir \
      --break-system-packages \
      --ignore-installed \
      pypcd4 \
      numpy \
      open3d

WORKDIR /ros2_ws
COPY docker/pcd_converter/ros2_src/ src/
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

COPY docker/isaacsim51/launch-isaac-jazzy.sh /opt/gearbox/launch-isaac-jazzy.sh
RUN chmod 0755 /opt/gearbox/launch-isaac-jazzy.sh

WORKDIR /isaac-sim
CMD ["/bin/bash"]
