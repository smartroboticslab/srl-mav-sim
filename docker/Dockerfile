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
ARG SSH_PRIV_KEY
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Install deps
RUN apt-get install -qy wget tmux

# Download srl-mav-sim
RUN mkdir $HOME/.ssh/
RUN echo "$SSH_PRIV_KEY" > $HOME/.ssh/id_rsa
RUN chmod 600 $HOME/.ssh/id_rsa
RUN touch $HOME/.ssh/known_hosts
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts

# BUILD COLCON WORKSPACE
WORKDIR $HOME
RUN mkdir -p $HOME/colcon_ws/src
RUN ls -a 
WORKDIR $HOME/colcon_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone -b ros2 https://github.com/smartroboticslab/srl-mav-sim.git \
    && cd srl-mav-sim \
    && git submodule update --init --recursive

RUN rm -rf $HOME/.ssh/id_rsa
COPY requirements.txt .
RUN apt-get install python3-pip -qy
RUN pip3 install -r requirements.txt
RUN cd $HOME/colcon_ws/src/srl-mav-sim/PX4-Autopilot \
    && bash ./Tools/setup/ubuntu.sh \
    && make px4_sitl

RUN cd $HOME/colcon_ws/src/srl-mav-sim/Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && sudo ldconfig /usr/local/lib/

RUN cd $HOME/colcon_ws/src/srl-mav-sim/PX4-Autopilot \
    && bash ./Tools/setup/ubuntu.sh \
    && make px4_sitl

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
