# Arguments
ARG ROS_BASE_IMAGE=humble-ros-base-jammy

# Select base image
FROM ros:$ROS_BASE_IMAGE

# Create a workspace directory
WORKDIR /ros2_ws_local
RUN mkdir src

# Clone and install micro ROS
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN apt-get update && rosdep update
RUN . /opt/ros/${ROS_DISTRO}/setup.sh; \
    rosdep install --from-paths src --ignore-src -y; \
    colcon build; \
    . install/local_setup.sh; \
    ros2 run micro_ros_setup create_agent_ws.sh; \
    ros2 run micro_ros_setup build_agent.sh

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    nano \
    tmux \
    gazebo \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-humble-image-tools \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory and mount the workspace volume

WORKDIR /ros2_ws
VOLUME ["/ros2_ws"]

# Source ROS setup file and local workspace
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws_local/install/setup.bash" >> /root/.bashrc


# Copy the entrypoint script into the image
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set the entry point
ENTRYPOINT ["/entrypoint.sh"]

# Default command (override with docker run command)
CMD ["bash"]
