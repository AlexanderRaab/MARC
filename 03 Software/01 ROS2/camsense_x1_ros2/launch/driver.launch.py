import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo 
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


''' 
Launch file for testing the SICK safeVisionary2
'''

 # Setup paths and file names
pkg_share = FindPackageShare(package='camsense_x1_ros2').find('camsense_x1_ros2')

config_path = os.path.join(pkg_share, 'config')
default_rviz_config = os.path.join(config_path, 'default.rviz')

# Setup parameter confinguration
configurable_parameters = [
    # Paths and files
    {'name': 'x',                       'default': "0",                                 'description': 'x-Position in world coordinate'},
    {'name': 'y',                       'default': "0",                                 'description': 'y-Position in world coordinate'},
    {'name': 'z',                       'default': "0",                                 'description': 'z-Position in world coordinate'},
    {'name': 'ro',                      'default': "0",                                 'description': 'roll-angle for base_link -> camera_frame'},
    {'name': 'pi',                      'default': "0",                                 'description': 'pitch-angle for base_link -> camera_frame'},
    {'name': 'ya',                      'default': "0",                                 'description': 'yaw-angle for base_link -> camera_frame'},
    {'name': 'base_link',               'default': "base_link",                         'description': 'Base link name'},
    {'name': 'lidar_frame',             'default': "lidar_link",                        'description': 'Lidar frame name'},
    {'name': 'dev',               	    'default': "/dev/ttyUSB0",                      'description': 'Serial device of the lidar'},
    {'name': 'topic',               	'default': "scan",                         	    'description': 'Topic the LaserScan data is published to'},
    {'name': 'namespace',               'default': "camsense",                         	'description': 'Namespace of the lidar'},
    {'name': 'use_rviz',               	'default': True,                         	    'description': 'Use rviz for visualization'},
    ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=str(param['default']), description=param['description']) for param in parameters]

def get_launch_config(parameters):
    return {param['name']: LaunchConfiguration(param['name']) for param in parameters}

def generate_launch_description():
    
    lc = get_launch_config(configurable_parameters)

    # Nodes
    start_driver = Node(
        package='camsense_x1_ros2',
        namespace=lc["namespace"],
        executable='driver',
        parameters=[{"dev": lc["dev"]}, {"frame": lc["lidar_frame"]}, {"topic": lc["topic"]}])

	
    # Static transform to base_link
    static_tf = Node(package = "tf2_ros", 
       executable = "static_transform_publisher",
       arguments = [lc["x"], lc["y"], lc["z"], lc["ro"], lc["pi"],lc["ya"], lc["base_link"], lc["lidar_frame"]])
                       

    # Launch RViz
    start_rviz = Node(
    	condition=IfCondition(lc['use_rviz']),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config])
				       			
		
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    declerations = declare_configurable_parameters(configurable_parameters)
    for param in declerations:
        ld.add_action(param)

    # Add any actions
    ld.add_action(start_driver)
    ld.add_action(start_rviz)
    ld.add_action(static_tf)

    return ld
