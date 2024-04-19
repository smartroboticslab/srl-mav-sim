FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Settings
ENV DEBIAN_FRONTEND=noninteractive

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USER=docker
ARG PASS=docker
ARG UID=1000
ARG GID=1000
ARG SSH_PRIV_KEY
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Install some basic tools and dependencies.
RUN apt-get -y update && apt-get install -y \
    iputils-ping \
    net-tools \
    wget \
    git \
    openssh-client \
    htop \
    libgoogle-glog-dev \
    python3-catkin-tools \
    ros-noetic-mavros-msgs \
    ros-noetic-depth-image-proc \
    ros-noetic-mavlink \
    ros-noetic-mavros

# Setup ssh keys.
RUN mkdir $HOME/.ssh/
RUN echo "$SSH_PRIV_KEY" > $HOME/.ssh/id_rsa
RUN chmod 600 $HOME/.ssh/id_rsa
RUN touch $HOME/.ssh/known_hosts
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts

# Create a catkin workspace.
WORKDIR $HOME
RUN mkdir -p $HOME/catkin_ws/src
WORKDIR $HOME/catkin_ws/src
RUN git clone https://github.com/smartroboticslab/srl-mav-sim.git \
    && cd srl-mav-sim \
    && git submodule update --init --recursive
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc

# Setup the simulation environment.
WORKDIR $HOME
ENV IGNITION_VERSION fortress
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

WORKDIR /
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh
RUN rm install_geographiclib_datasets.sh

COPY requirements.txt .
RUN apt-get install python3-pip -qy
RUN pip3 install -r requirements.txt

WORKDIR $HOME/catkin_ws/src/srl-mav-sim
RUN ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
# WORKDIR $HOME/catkin_ws/src/srl-mav-sim/PX4-Autopilot
# RUN DONT_RUN=1 make px4_sitl gazebo

# Remove SSH keys.
RUN rm -rf $HOME/.ssh/id_rsa

# Set working directory.
WORKDIR $HOME
