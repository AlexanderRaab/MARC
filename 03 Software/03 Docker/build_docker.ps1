set-variable -name IMAGE_NAME -value marc_ros2:humble
set-variable -name ROS_BASE_IMAGE -value humble-ros-base-jammy

docker build  `
    --build-arg ROS_BASE_IMAGE=${ROS_BASE_IMAGE}  `
    -t ${IMAGE_NAME} .