ARG ROS_DISTRO=humble

###############################
# Base image for xArm control #
###############################

FROM osrf/ros:${ROS_DISTRO}-desktop as base 
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL [ "/bin/bash" , "-c" ]

# Create colcon workspace with external dependencies
RUN mkdir -p /xarm_ws/src
WORKDIR /xarm_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the workspace
WORKDIR /xarm_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update -y \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-moveit-* \
    ros-${ROS_DISTRO}-topic-based-ros2-control
    # ros-${ROS_DISTRO}-gazebo-ros2-control \
    # ros-${ROS_DISTRO}-ros2-control

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

##################################
# Overlay image for xArm control #
##################################

FROM base AS overlay

# Create overlay colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./xarm_description/ /overlay_ws/src/xarm_description/
COPY ./xarm_moveit_config/ /overlay_ws/src/xarm_moveit_config/
COPY ./xarm_gazebo/ /overlay_ws/src/xarm_gazebo/
COPY ./Hiwonder_xArm_ESP32 /overlay_ws/src/
COPY ./xarm_support/ /overlay_ws/src/xarm_support/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /xarm_ws/install/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

# Setup entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

######################################
# Development image for xArm control #
######################################

FROM overlay AS dev 

# Change USERNAME to the host USERNAME
ARG USERNAME=diego
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    gdbserver \
    nano

# Create new user and home directory
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid ${USER_GID} --gid ${USER_UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

RUN sudo usermod -aG dialout ${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${USER_UID}:${USER_GID} /overlay_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc