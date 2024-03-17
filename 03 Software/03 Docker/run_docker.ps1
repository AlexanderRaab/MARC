Set-Variable -Name IMAGE_NAME -value marc_ros2:humble
Set-Variable -Name CON_NAME   -Value marc_ros2_con
Set-Variable -Name DISPLAY  -value "host.docker.internal:0.0"
Set-Variable -Name CUR_PATH -value (Get-Location)
Set-Variable -Name HOST_WS_PATH -value "${CUR_PATH}\ros2_ws"

$XServer = Get-Process -Name VcXsrv -ErrorAction SilentlyContinue

if($XServer) {
    echo "X Server is already running"
} else {
    Start-Process -FilePath 'C:\Program Files\VcXsrv\xlaunch.exe' -ArgumentList "-run .\config_ROS.xlaunch"
}

docker run -it --rm -e DISPLAY=$DISPLAY `
    --name $CON_NAME `
    --net=host `
    --mount type=bind,source=$HOST_WS_PATH,target=/ros2_ws `
    "${IMAGE_NAME}"