FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------------------------------------
# 1. Base + ROS2 + outils (1 seule layer optimisée)
# --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    && add-apt-repository universe \
    && locale-gen en_US en_US.UTF-8 \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-robot-localization \
        ros-humble-rosbridge-server \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-pip \
        git \
        build-essential \
        libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 2. rosdep
# --------------------------------------------------
RUN rosdep init || true && rosdep update

# --------------------------------------------------
# 3. UNDERLAY WORKSPACE (clé de ton setup)
# --------------------------------------------------
WORKDIR /opt/underlay_ws
RUN mkdir -p src

# Copier uniquement le repos file (cache ++)
COPY underlay.repos .

# Import dépendances
RUN vcs import src < underlay.repos

# Fix rosbridge tests
RUN touch /opt/underlay_ws/src/rosbridge_suite/rosbridge_test_msgs/COLCON_IGNORE

# Installer dépendances system (cache si repos inchangé)
RUN apt-get update && \
    rosdep install -i -r --from-paths src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# Build underlay (stable → cache Docker efficace)
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DBUILD_TESTING=OFF"

# --------------------------------------------------
# 4. Workspace DEV (monté en volume)
# --------------------------------------------------
WORKDIR /workspace

# pip deps (cache indépendant du code)
COPY requirements_xavier.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# --------------------------------------------------
# 5. Entrypoint propre (multi-overlay ROS)
# --------------------------------------------------
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
