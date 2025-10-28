ARG TARGET_ROS_VERSION=jazzy
FROM ros:${TARGET_ROS_VERSION}

ARG TARGET_ROS_VERSION=jazzy
ENV TARGET_ROS_VERSION=${TARGET_ROS_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# ---  Set up locale and add universe repository ---
RUN apt-get update && apt-get install -y locales software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# ---  Add the official Gazebo repository ---
RUN apt-get update && apt-get install -y curl gpg lsb-release && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    rm -rf /var/lib/apt/lists/*

# ---  Install all dependencies, including ROS Desktop and MAVROS ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Full ROS Desktop experience
    ros-${TARGET_ROS_VERSION}-desktop \
    # MAVROS packages
    ros-${TARGET_ROS_VERSION}-depth-image-proc \
    ros-${TARGET_ROS_VERSION}-mavlink \
    ros-${TARGET_ROS_VERSION}-mavros \
    ros-${TARGET_ROS_VERSION}-mavros-msgs \
    # Gazebo and its development files
    gz-harmonic \
    libgz-sim8-dev \
    # ROS <-> Gazebo integration
    ros-${TARGET_ROS_VERSION}-ros-gz \
    ros-${TARGET_ROS_VERSION}-ros-gz-interfaces \
    # Standard development tools
    git \
    vim \
    tmux \
    ros-dev-tools \
    python3-vcstool \
    python3-kconfiglib \
    # Other system dependencies
    libgflags-dev \
    libtinyxml-dev \
    libboost-filesystem-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgz-common6-dev \
    libsuitesparse-dev \
    libompl-dev \
    libgoogle-glog-dev \
    # CLI11 library for CMake find_package
    libcli11-dev \
    # GUI/OpenGL libraries
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# ---  Install MAVROS GeographicLib Datasets ---
# This is required for MAVROS to handle geographic coordinate conversions.
RUN cd /opt/ros/${TARGET_ROS_VERSION}/lib/mavros && bash ./install_geographiclib_datasets.sh

# Create a ROS 2 workspace
RUN mkdir -p /ros2_ws/src/srl-mav-sim
WORKDIR /ros2_ws

# Copy and set up the entrypoint script
COPY . /ros2_ws/src/srl-mav-sim
# remove any existing build directories to avoid conflicts
RUN rm -rf /ros2_ws/src/srl-mav-sim/srl_sim_gz/px4-gzsim-plugins/build /ros2_ws/src/PX4-Autopilot/build
# build the workspace
RUN apt-get update && \
    . /opt/ros/${TARGET_ROS_VERSION}/setup.sh && \
    rm -rf /ros2_ws/build /ros2_ws/log /ros2_ws/install && \
    colcon build && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

# Set the default command to start a shell
CMD ["bash"]
