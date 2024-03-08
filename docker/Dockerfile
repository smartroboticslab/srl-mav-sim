FROM ubuntu:focal
SHELL ["/bin/bash", "-c"]

# Settings
ENV ARCH=amd64
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

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

# Install deps
RUN apt-get install -qy \
  git \
  tree \
  htop \
  tmux \
  exuberant-ctags \
  automake \
  cmake \
  ccache \
  gcc \
  clang \
  clang-format \
  clang-tidy \
  protobuf-compiler \
  libpython3-dev \
  pylint \
  yapf3

# Install ROS
RUN apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install curl gnupg -qy
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update
RUN apt-get install ros-${ROS_DISTRO}-desktop -qy

# Install Gazebo-Fortress
RUN apt-get install wget -qy
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install ignition-fortress -qy

# Install MAVROS
RUN apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras -qy

# Install additional tools and libraries
RUN apt-get install -qy \
  python3-pip \
  python3-numpy \
  python3-scipy \
  python3-pandas \
  python3-seaborn \
  python3-matplotlib \
  python3-setuptools \
  python3-catkin-tools \
  libeigen3-dev \
  libopencv-dev

# Switch to user
USER ${USER}

# Setup catkin workspace
WORKDIR $HOME
RUN mkdir -p $HOME/catkin_ws/src
RUN cd $HOME/catkin_ws \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && catkin init \
  && catkin build

# Download srl-mav-sim
RUN mkdir $HOME/.ssh/
RUN echo "$SSH_PRIV_KEY" > $HOME/.ssh/id_rsa
RUN chmod 600 $HOME/.ssh/id_rsa
RUN touch $HOME/.ssh/known_hosts
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts
WORKDIR $HOME/catkin_ws/src
RUN git clone --recurse-submodules git@github.com:smartroboticslab/srl-mav-sim.git
RUN git clone --recurse-submodules git@github.com:smartroboticslab/mpc_ros.git
RUN git clone git@github.com:smartroboticslab/mav_ekf.git
RUN rm -rf $HOME/.ssh/id_rsa

# Install srl-mav-sim
WORKDIR $HOME/catkin_ws/src
RUN pip3 install kconfiglib
RUN apt install -qy ant openjdk-11-jdk
USER root
RUN ./srl-mav-sim/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
USER ${USER}

# Install GeographicLib datasets
WORKDIR $HOME/catkin_ws
USER root
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN sudo bash ./install_geographiclib_datasets.sh
RUN rm install_geographiclib_datasets.sh
USER ${USER}

# Build catkin workspace
WORKDIR $HOME/catkin_ws
USER root
RUN apt-get install \
  libgflags-dev \
  libgoogle-glog-dev \
  ros-${ROS_DISTRO}-gazebo-msgs \
  libignition-gazebo3-* \
  libignition-transport8-* \
  -qy
USER ${USER}
RUN env IGNITION_VERSION=fortress catkin build -DCMAKE_BUILD_TYPE=Release

# Tools
USER root
WORKDIR $HOME
# -- Install Neovim
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux64.tar.gz \
  && sudo rm -rf /opt/nvim \
  && sudo tar -C /opt -xzf nvim-linux64.tar.gz \
  && rm nvim-linux64.tar.gz \
  && echo 'export PATH=$PATH:/opt/nvim-linux64/bin' > $HOME/.bashrc
# -- Install vim and vifm
RUN apt-get install -qy vim vifm
# -- Add sane tmux configs
RUN echo "set -g default-command /bin/bash" >> $HOME/.tmux.conf
RUN echo "set -g mouse on" >> $HOME/.tmux.conf
RUN echo "bind-key h select-pane -L" >> $HOME/.tmux.conf
RUN echo "bind-key j select-pane -D" >> $HOME/.tmux.conf
RUN echo "bind-key k select-pane -U" >> $HOME/.tmux.conf
RUN echo "bind-key l select-pane -R" >> $HOME/.tmux.conf
USER ${USER}


# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]
