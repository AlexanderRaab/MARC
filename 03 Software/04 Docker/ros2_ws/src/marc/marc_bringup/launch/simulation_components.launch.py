import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#########################################################################
###
### Launch file for bringing up all purely 
### simulation related utilities for MARC
###
#########################################################################


# Initialize default paths
pkg_share_bringup = get_package_share_directory('marc_bringup')
pkg_share_description = get_package_share_directory('marc_description')
pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

default_robot_model_path = os.path.join(pkg_share_description, 'src', 'robot', 'MARC.urdf.xacro')
default_world_file_path = os.path.join(pkg_share_bringup, 'src', 'worlds', 'mt_og5.world')

# Setup parameter confinguration
configurable_parameters = [
    # Paths and files
    {'name': 'world_file',                    	'default': default_world_file_path,         			'description': 'Gazebo world file'},
    {'name': 'namespace',						'default': "",											'description': 'Application namespace'}
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

	# --- Simulation
    # Start Gazebo server
	start_gazebo_server = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
		launch_arguments={'world': launch_config['world_file']}.items())

    # Start Gazebo client    
	start_gazebo_client = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),)

	# Spawn robot in gazebo
	spawn_robot = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-entity', 'MARC', '-topic', 'robot_description'],
		output='screen')

	# Declare the launch options
	declerations = declare_configurable_parameters(configurable_parameters)
	for param in declerations:
		ld.add_action(param)

	# Add any actions
	ld.add_action(start_gazebo_server)
	ld.add_action(start_gazebo_client)
	ld.add_action(spawn_robot)
	return ld
