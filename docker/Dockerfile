# Start from official ROS 2 Humble Desktop image (Ubuntu 22.04 base)
FROM osrf/ros:humble-desktop-full

# Install OpenCV (C++ libs + Python bindings)
RUN apt-get update && apt-get install -y \
    gedit libopencv-dev python3-opencv libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev libdw-dev ros-humble-mavros* ros-humble-pcl* ros-humble-rmw-zenoh-cpp\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y libeigen3-dev  \     
    && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Source ROS setup automatically

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/phoenix/install/setup.bash" >> /etc/bash.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> /etc/bash.bashrc
RUN echo "alias zenohd='ros2 run rmw_zenoh_cpp rmw_zenohd'" >> /etc/bash.bashrc
RUN echo "alias start='source install/setup.bash && ros2 launch mission_planner click_demo.launch.py'" >> /etc/bash.bashrc
RUN echo "alias build='colcon build --symlink-install --packages-select mars_quadrotor_msgs && source install/setup.bash && colcon build --symlink-install'" >> /etc/bash.bashrc

RUN echo "bash /home/phoenix/src/SUPER/scripts/select_ros_version.sh ROS2" >> /etc/bash.bashrc
RUN echo "echo -e \"Run 'build' to build ROS2 ws, and 'start' to launch the ROS2 point click demo.\"" >> /etc/bash.bashrc


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