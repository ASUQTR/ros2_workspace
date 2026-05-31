FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

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
        ros-humble-flexbe-* \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-pip \
        python3-tk \
        git \
        build-essential \
        libserial-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /opt/underlay_ws
RUN mkdir -p src

COPY underlay.repos .

RUN vcs import src < underlay.repos

RUN touch /opt/underlay_ws/src/rosbridge_suite/rosbridge_test_msgs/COLCON_IGNORE

RUN apt-get update && \
    rosdep install -i -r --from-paths src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DBUILD_TESTING=OFF"

WORKDIR /workspace

COPY requirements_desktop.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --upgrade -r /tmp/requirements.txt && \
    rm -rf /usr/lib/python3/dist-packages/matplotlib \
           /usr/lib/python3/dist-packages/matplotlib-*.egg-info \
           /usr/lib/python3/dist-packages/matplotlib-*.dist-info \
           /usr/lib/python3/dist-packages/matplotlib-*.pth \
           /usr/lib/python3/dist-packages/mpl_toolkits

# Force pip packages before system packages so matplotlib>=3.7 takes precedence over apt's 3.5
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH

# Source ROS + workspace + aliases in every interactive terminal (docker exec bash)
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/underlay_ws/install/setup.bash' >> ~/.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> ~/.bashrc && \
    echo '[ -f /workspace/asuqtr_aliases.sh ] && source /workspace/asuqtr_aliases.sh' >> ~/.bashrc

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
