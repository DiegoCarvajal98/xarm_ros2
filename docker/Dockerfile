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
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

##################################
# Overlay image for xArm control #
##################################

FROM base AS overlay

# Create overlay colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
RUN source /xarm_ws/install/setup.bash \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
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

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${USER_UID}:${USER_GID} /overlay_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc

##########################################################################################################
## Create the user
#RUN groupadd --gid $USER_GID $USERNAME \
#    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#    #
#    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
#    && apt-get update \
#    && apt-get install -y sudo \
#    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#    && chmod 0440 /etc/sudoers.d/$USERNAME
#RUN apt-get update && apt-get upgrade -y
#RUN apt-get install -y python3-pip \
#    && apt-get install -y git
#
#ENV SHELL /bin/bash
#
#WORKDIR /root/xarm_ws
#
#RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
#
#RUN apt-get update \
#    && rosdep update \
#    && rosdep install --from-paths src --ignore-src -r -y 
#
#RUN colcon build \
#    && . install/setup.bash
#
## [Optional] Set the default user. Omit if you want to keep the default as root.
#USER $USERNAME
#
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
#CMD ["/bin/bash"]