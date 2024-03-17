IMAGE_NAME=marc_ros2:humble
ROS_BASE_IMAGE=humble-ros-base-jammy


docker build \
  --build-arg ROS_BASE_IMAGE=$ROS_BASE_IMAGE \
  -t $IMAGE_NAME .
