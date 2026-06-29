FROM ros:humble-ros-base
ARG DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV ROS_LOCALHOST_ONLY=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-gui curl wget less \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-$DISTRO-turtlebot3 \
    ros-$DISTRO-turtlebot3-simulations \
    ros-$DISTRO-gazebo-ros-pkgs \
    ros-$DISTRO-rviz2 \
    ros-$DISTRO-tf2-tools \
    ros-$DISTRO-tf2-geometry-msgs \
    ros-$DISTRO-rqt \
    ros-$DISTRO-rqt-common-plugins \
    ros-$DISTRO-plotjuggler-ros \
    ros-$DISTRO-joint-state-publisher \
    ros-$DISTRO-angles \
    ros-$DISTRO-laser-geometry \
    ros-$DISTRO-laser-filters \
    ros-$DISTRO-twist-mux \
    ros-$DISTRO-magic-enum \
    ros-$DISTRO-robot-state-publisher \
    ros-$DISTRO-xacro \
    tmux vim nano emacs \
    bash-completion htop \
    gdb gdbserver valgrind \
    iputils-ping net-tools openssh-client \
    doxygen clang-format clang-tidy \
    jq rsync unzip psmisc \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

RUN git config --global --add safe.directory "*"

RUN echo "source /opt/ros/$DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source /root/install/setup.bash 2>/dev/null || true" >> /root/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Copy source and build packages
COPY src /root/src/autonomy_dev/src
WORKDIR /root
RUN /bin/bash -c "source /opt/ros/$DISTRO/setup.bash && \
    colcon build --base-paths src/autonomy_dev/src"
COPY scripts /root/src/autonomy_dev/scripts


CMD ["/bin/bash"]