IMAGE_NAME=marc_ros2:humble

HOST_WS_PATH="$(pwd)/ros2_ws"

docker run -it --rm\
  --net=host \
  --mount type=bind,source=$HOST_WS_PATH,target=/ros2_ws \
  --volume /dev:/dev \
  --privileged \
  --env DISPLAY=${DISPLAY} \
  ${IMAGE_NAME}
