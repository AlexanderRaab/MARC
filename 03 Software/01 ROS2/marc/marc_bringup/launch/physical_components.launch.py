import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#########################################################################
###
### Launch file for bringing up all utilities necessary for 
### running the physical MARC
###
#########################################################################


# Initialize default paths
pkg_share_bringup = get_package_share_directory('marc_bringup')

default_base_device = "/dev/ttyACM0"
default_base_device_baudrate = 115200
default_lidar_device = "/dev/ttyUSB0"
default_camera_device = "/dev/video0"
default_camera_framerate = 10.


# Setup parameter confinguration
configurable_parameters = [
    # Paths and files
	{'name': 'use_plattform',					'default': True,										'description': 'Activate the differential drive plattform'},
    {'name': 'use_lidar',						'default': True,										'description': 'Activate the main lidar scanner'},
    {'name': 'use_camera',						'default': True,										'description': 'Activate the main camera'},
    {'name': 'base_device',                    	'default': default_base_device,                			'description': 'Serial device of the mobile base platform'},
    {'name': 'base_baudrate',                  	'default': default_base_device_baudrate,               	'description': 'Serial baudrate of the mobile base platform'},
    {'name': 'lidar_device',                    'default': default_lidar_device,                		'description': 'Serial device of the lidar'},
    {'name': 'camera_device',                   'default': default_camera_device,                		'description': 'Device of the camera input'},
    {'name': 'camera_framerate',                'default': default_camera_framerate,                	'description': 'Framerate in Hz'},
    {'name': 'namespace',						'default': "",											'description': 'Application namespace'},
    ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=str(param['default']), description=param['description']) for param in parameters]

def get_launch_config(parameters):
    return {param['name']: LaunchConfiguration(param['name']) for param in parameters}

def generate_launch_description():
    
	# Create the launch description
	ld = LaunchDescription()
        
	# Create launch config
	launch_config = get_launch_config(configurable_parameters)
    
	# --- Physical launch
    # MicroROS agent for base platform
	start_mobile_base_agent = Node(
        condition=IfCondition(launch_config['use_plattform']),
		namespace=launch_config['namespace'],
		package='micro_ros_agent',
		executable='micro_ros_agent',
		name='mobile_base_agent',
		arguments=["serial", '--dev', launch_config['base_device'], '-b', launch_config['base_baudrate']])
        
	start_camsense_driver = Node(
        condition=IfCondition(launch_config['use_lidar']),
        package='camsense_x1_ros2',
        namespace=launch_config["namespace"],
        executable='driver',
        name='camsense_x1_node',
        parameters=[{"dev": launch_config["lidar_device"], "frame": "lidar_sensor_link", "topic": "scan"}])
    
	start_camera_driver = Node(
        condition=IfCondition(launch_config['use_camera']),
        package='image_tools',
        executable='cam2image',
        name='front_camera_node',
        output="log",
        namespace=launch_config["namespace"],
        remappings=[('/image', '/front_camera/image')],
        parameters=[{'video_device': launch_config["camera_device"], 'frequency': launch_config["camera_framerate"], 'frame_id': "front_camera_link"}]
        )

	# Declare the launch options
	declerations = declare_configurable_parameters(configurable_parameters)
	for param in declerations:
		ld.add_action(param)

	# Add any actions
	ld.add_action(start_mobile_base_agent)
	ld.add_action(start_camsense_driver)
	ld.add_action(start_camera_driver)

	return ld
