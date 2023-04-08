FROM ros:kinetic-ros-core

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    default-jdk \
    gradle  \
    maven \
    wget \
    ros-kinetic-catkin \
    ros-kinetic-rospack \
    python-wstool \
    libc6-dev-i386 \
    libc6-x32 \
    && rm -rf /var/lib/apt/lists/

ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64

RUN apt-get update && apt-get install -y \
    ros-kinetic-world-canvas-msgs \
    ros-kinetic-concert-service-msgs \
    ros-kinetic-ar-track-alvar-msgs \
    ros-kinetic-gateway-msgs \
    ros-kinetic-rocon-device-msgs \
    ros-kinetic-rocon-app-manager-msgs \
    ros-kinetic-scheduler-msgs \
    ros-kinetic-rocon-tutorial-msgs \
    ros-kinetic-rocon-interaction-msgs \
    ros-kinetic-yocs-msgs \
    ros-kinetic-concert-msgs \
    ros-kinetic-move-base-msgs \
    ros-kinetic-tf2-msgs \
    && rm -rf /var/lib/apt/lists/

RUN mkdir -p /root/rosjava/src

WORKDIR /root/rosjava
RUN wstool init -j4 /root/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall

COPY rosjava_bootstrap.patch /root/rosjava/src/rosjava_bootstrap
RUN cd /root/rosjava/src/rosjava_bootstrap && \
    git apply rosjava_bootstrap.patch --reject --whitespace=fix 

COPY rosjava_core.patch /root/rosjava/src/rosjava_core
RUN cd /root/rosjava/src/rosjava_core && \
    git apply rosjava_core.patch --reject --whitespace=fix 

COPY zeroconf_jmdns_suite.patch /root/rosjava/src/zeroconf_jmdns_suite
RUN cd /root/rosjava/src/zeroconf_jmdns_suite && \
    git apply zeroconf_jmdns_suite.patch --reject --whitespace=fix 

RUN source /opt/ros/kinetic/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    catkin_make

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
