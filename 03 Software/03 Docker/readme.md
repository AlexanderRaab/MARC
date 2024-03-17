# Docker
This directory includes all files for running the ROS2 project inside a Docker container.

**Attention: The docker environment is still under development and may not work as intended!**
**The implementation is currently only tested for Windows 10 and requires [vcxsrv](https://uni-tuebingen.de/en/fakultaeten/wirtschafts-und-sozialwissenschaftliche-fakultaet/faecher/fachbereich-wirtschaftswissenschaft/wirtschaftswissenschaft/fb-wiwi/einrichtungen-wirtschaftswissenschaft/wiwi-it/services/services/computing-asp/tools/x-server/vcxsrv/)**

## Usage
Paste all custom ROS2 packages to [ros2_ws/src](ros2_ws/src).

On Windows, build the docker container with

    ./build_docker.ps1

Use 

    ./run_docker.ps1

to run the container and

    ./attach_docker.ps1

to open new shell windows.
