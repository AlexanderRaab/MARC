#!/bin/bash

cd /ros2_ws

# Source ROS, build workspace and source overlay
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source ./install/local_setup.bash

# Set the session name
SESSION="ROS2"

# Check if session exists
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ -z "$SESSIONEXISTS" ]; then
    # Start New Session with our name
    tmux new-session -d -s $SESSION

    # Pane 0
    tmux rename-window -t $SESSION:0 '0'
    tmux send-keys -t $SESSION:0 "ros2 info" 'C-m'

    # Pane 1
    tmux new-window -t $SESSION:1 -n '1'
    tmux send-keys -t $SESSION:1 "ros2 launch marc_bringup MARC.launch.py simulation:=False visual:=False use_sim_time:=False"

    # Pane 2
    tmux new-window -t $SESSION:2 -n '2'
    tmux send-keys -t $SESSION:2 "run micro_ros_agent micro_ros_agent serial - serial --dev '/dev/ttyACM0' -b 115200"

    # Pane 3
    tmux new-window -t $SESSION:3 -n '3'
    tmux send-keys -t $SESSION:3 "ros2 topic pub --rate 100 cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0, z: 0}}'"

    # Pane 4
    tmux new-window -t $SESSION:4 -n '4'
    tmux send-keys -t $SESSION:4 "ros2 topic list"
fi

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux attach-session -t $SESSION

# Execute the CMD or override it with "bash" if not specified
exec "$@"
