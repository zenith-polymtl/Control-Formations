# syntax=docker/dockerfile:1.6
ARG ROS_DISTRO=humble
FROM --platform=$TARGETPLATFORM ros:${ROS_DISTRO}-ros-base

ARG TARGETPLATFORM
ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Update & install ROS/colcon build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install core Python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-numpy-dev \
    && python3 -m pip install --upgrade pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN python3 -m pip install -U \
    zenmav \
    geopy \
    pymavlink

RUN python3 -m pip install -U \
    colcon-common-extensions \
    vcstool \
    rosdep \
    pytest-cov \
    pytest-repeat \
    pytest-rerunfailures \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    setuptools==65.7.0

# Install ROS visualization & extra tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavlink \
    geographiclib-tools \
    ros-humble-demo-nodes-cpp \
    ros-dev-tools \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*


# Install FastAPI and Uvicorn
RUN python3 -m pip install fastapi uvicorn[standard]

# Set up workspace
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
RUN pip install --no-cache-dir matplotlib
RUN pip install --no-cache-dir pandas

# put this very last so earlier layers stay cached
ARG ZENMAV_REFRESH=0
RUN python3 -m pip install --no-cache-dir -U zenmav && echo "refresh=$ZENMAV_REFRESH"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
 && apt-get install -y --no-install-recommends \
       ros-humble-tf-transformations \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --ignore-installed "transforms3d>=0.4.2"

WORKDIR /ros2_ws

COPY ros2_ws/src ./src

# Build the workspace using bash explicitly
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build"]

# Add sourcing to .bashrc using bash
RUN ["/bin/bash", "-c", "echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc"]
RUN ["/bin/bash", "-c", "echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc"]