FROM dynorobotics/balena-amd64-node-ros2:eloquent-gpu

RUN apt-get update && apt-get install -y \
  can-utils \
  libsocketcan-dev \
  && rm -rf /var/likb/apt/lists/*

# Clone ros2-web-bridge
WORKDIR /opt  
RUN git clone https://github.com/RobotWebTools/ros2-web-bridge.git --branch 0.2.7

# install web bridge
WORKDIR /opt/ros2-web-bridge
RUN . $ROS2_WS/install/local_setup.sh   && npm install

ENV ROS2_WS /opt/ros2_ws

ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS/src

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src
WORKDIR $DEPENDENCIES_WS

# clone dependency ros package repos
COPY ros_dependencies.repos $DEPENDENCIES_WS/ros_dependencies.repos
RUN vcs import src < $DEPENDENCIES_WS/ros_dependencies.repos

# install dependency ros package dependencies
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh &&     apt-get update &&     rosdep install -q -y       --from-paths         src       --ignore-src       --rosdistro $CHOOSE_ROS_DISTRO     && rm -rf /opt/ros     && rm -rf /var/lib/apt/lists/*

# build dependency ros package source
ARG CMAKE_BUILD_TYPE=Debug
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/local_setup.sh &&      colcon build        --merge-install        --cmake-args          -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# copy this repo
WORKDIR $OVERLAY_WS
COPY ./ src

# install ros package dependencies
WORKDIR $OVERLAY_WS
RUN . $ROS2_WS/install/local_setup.sh \
  && . $DEPENDENCIES_WS/install/local_setup.sh \
  && apt-get update \
  && rosdep install -q -y \
    --from-paths src \
    --ignore-src \
    --rosdistro $CHOOSE_ROS_DISTRO \
    --skip-keys "filters diagnostic_updater" \
  && rm -rf /var/lib/apt/lists/*

# build package source
ARG CMAKE_BUILD_TYPE=Debug
WORKDIR $OVERLAY_WS
RUN bash -c ". $ROS2_WS/install/local_setup.bash && . $DEPENDENCIES_WS/install/local_setup.bash && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE"

# build messages for ros2-web-bridge
WORKDIR /opt/ros2-web-bridge
RUN bash -c ". $ROS2_WS/install/local_setup.bash && . $DEPENDENCIES_WS/install/local_setup.bash && . $OVERLAY_WS/install/local_setup.bash && rm -r ./node_modules/rclnodejs && npm install"

WORKDIR $OVERLAY_WS

# NOTE(sam): Needed for autocomplete (does not seem to work with only entrypoint sourcing...)
RUN echo "source $ROS2_WS/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source $DEPENDENCIES_WS/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source $OVERLAY_WS/install/local_setup.bash" >> $HOME/.bashrc

COPY tools $OVERLAY_WS/

# source workspace from entrypoint if available
COPY ros_entrypoint.sh /
