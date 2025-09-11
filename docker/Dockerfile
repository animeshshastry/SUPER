# Start from official ROS 2 Humble Desktop image (Ubuntu 22.04 base)
FROM osrf/ros:humble-desktop-full

# Install OpenCV (C++ libs + Python bindings)
RUN apt-get update && apt-get install -y \
    libopencv-dev python3-opencv libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev libdw-dev ros-humble-mavros* ros-humble-pcl*\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y libeigen3-dev  \     
    && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Source ROS setup automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "bash /home/phoenix/src/scripts/select_ros_version.sh ROS2" >> ~/.bashrc

# Create a user (example: phoenix) with a home directory
ARG USERNAME=phoenix
ARG USER_UID=1001
ARG USER_GID=1001

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set permissions for the user
WORKDIR /home/$USERNAME
USER $USERNAME

ENV ROS2_WS /home/phoenix
WORKDIR $ROS2_WS

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]