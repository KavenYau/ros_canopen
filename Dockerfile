FROM dynorobotics/balena-amd64-ros2:crystal

ENV DEBIAN_FRONTEND noninteractive

ENV ROS2_WS /opt/ros2_ws

ENV CANOPEN_WS /opt/canopen_ws
RUN mkdir -p $CANOPEN_WS/src
WORKDIR $CANOPEN_WS/src

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src
WORKDIR $DEPENDENCIES_WS

# install apt packages
RUN apt-get update \
  && apt-get install -y libboost-dev libboost-all-dev \
  && rm -rf /var/lib/apt/lists/*

# clone dependency ros package repos
COPY ros_dependencies.repos $DEPENDENCIES_WS/ros_dependencies.repos
RUN vcs import src < $DEPENDENCIES_WS/ros_dependencies.repos

# install dependency ros package dependencies
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
      --rosdistro $CHOOSE_ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# build dependency ros package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE


# install ros package dependencies
WORKDIR $CANOPEN_WS

# copy ros package repo
COPY ./ src/ros_canopen/

# NOTE(sam): don't know why it does not pick up keys from dependencies
RUN . $DEPENDENCIES_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
      --rosdistro $CHOOSE_ROS_DISTRO \
      --skip-keys "canopen_msgs filters diagnostic_updater controller_interface controller_manager hardware_interface" \
    && rm -rf /var/lib/apt/lists/*

# build package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $CANOPEN_WS
RUN . $DEPENDENCIES_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

RUN echo "source /opt/ros2_ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "source /opt/dependencies_ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "source /opt/canopen_ws/install/setup.bash" >> $HOME/.bashrc

# source workspace from entrypoint if available
COPY ros_entrypoint.sh /
