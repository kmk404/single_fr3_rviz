FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpoco-dev \
    libusb-1.0-0 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    python3-opencv \
    python3-scipy \
    ros-dev-tools \
    ros-jazzy-controller-manager \
    ros-jazzy-cv-bridge \
    ros-jazzy-std-srvs \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY franka_jazzy.repos /tmp/franka_jazzy.repos
RUN mkdir -p /opt/franka_ws/src \
    && vcs import /opt/franka_ws/src < /tmp/franka_jazzy.repos --recursive

RUN touch /opt/franka_ws/src/franka_ros2/realtime_tools/COLCON_IGNORE

RUN source /opt/ros/jazzy/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --ignore-src --rosdistro jazzy -r -y \
       --dependency-types build \
       --dependency-types buildtool \
       --dependency-types build_export \
       --dependency-types buildtool_export \
       --dependency-types exec \
       --from-paths \
         /opt/franka_ws/src/libfranka \
         /opt/franka_ws/src/franka_description \
         /opt/franka_ws/src/franka_ros2/franka_msgs \
         /opt/franka_ws/src/franka_ros2/franka_semantic_components \
         /opt/franka_ws/src/franka_ros2/franka_hardware \
         /opt/franka_ws/src/franka_ros2/franka_gripper \
         /opt/franka_ws/src/franka_ros2/franka_robot_state_broadcaster \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
    && cd /opt/franka_ws \
    && colcon build --merge-install \
       --packages-up-to franka_hardware franka_gripper franka_robot_state_broadcaster \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTING=OFF \
         -DBUILD_TESTS=OFF \
         -DBUILD_EXAMPLES=OFF

RUN set -eux; \
    existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    if [[ -n "${existing_group}" && "${existing_group}" != "ros" ]]; then \
      groupmod --new-name ros "${existing_group}"; \
    elif [[ -z "${existing_group}" ]]; then \
      groupadd --gid "${USER_GID}" ros; \
    fi; \
    existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    if [[ -n "${existing_user}" && "${existing_user}" != "ros" ]]; then \
      usermod --login ros --home /home/ros --move-home --gid "${USER_GID}" "${existing_user}"; \
    elif [[ -z "${existing_user}" ]]; then \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash ros; \
    fi; \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc; \
    echo "source /opt/franka_ws/install/setup.bash" >> /home/ros/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ros
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
