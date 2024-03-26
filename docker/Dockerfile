FROM ros:humble-perception-jammy
SHELL ["/bin/bash", "-c"]

# Settings
ENV ARCH=amd64
ENV DEBIAN_FRONTEND=noninteractive

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USER=docker
ARG PASS=docker
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Install deps
RUN apt-get install -qy wget tmux

# BUILD PX4
WORKDIR $HOME
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN cd PX4-Autopilot \
  && bash ./Tools/setup/ubuntu.sh \
  && make px4_sitl

# BUILD MICRO-XRCE-DDS-AGENT
WORKDIR $HOME
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  && cd Micro-XRCE-DDS-Agent \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && sudo make install \
  && sudo ldconfig /usr/local/lib/

# BUILD COLCON WORKSPACE
WORKDIR $HOME
RUN mkdir -p $HOME/colcon_ws/src
WORKDIR $HOME/colcon_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git
RUN git clone https://github.com/PX4/px4_ros_com.git
WORKDIR $HOME/colcon_ws
RUN source /opt/ros/humble/setup.bash && colcon build

# Install nvim
RUN wget https://github.com/neovim/neovim/releases/latest/download/nvim.appimage \
  && chmod u+x nvim.appimage \
  && mkdir -p /opt/nvim \
  && mv nvim.appimage /usr/local/nvim

# Install dotfiles
# USER $USER
# WORKDIR $HOME
# RUN git clone https://github.com/chutsu/dotfiles.git \
#   && cd dotfiles \
#   && make init \
#   && make gitconfig \
#   && make nvim \
#   && make vifm

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]
