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
### Launch file for bringing up all core 
###  utilities for MARC
###
#########################################################################


# Initialize default paths
pkg_share_bringup = get_package_share_directory('marc_bringup')
pkg_share_description = get_package_share_directory('marc_description')
pkg_config = os.path.join(pkg_share_bringup, 'config')
default_robot_model_path = os.path.join(pkg_share_description, 'src', 'robot', 'MARC.urdf.xacro')
default_localization_config_path = os.path.join(pkg_config, 'robot_localization_ekf.yaml')


# Setup parameter confinguration
configurable_parameters = [
	# Paths and files
    {'name': 'robot_localization_file_path',	'default': default_localization_config_path,         	'description': 'Path to the config file for the robot_localization package'},
    {'name': 'robot_model',                   	'default': default_robot_model_path,         			'description': 'Robot model file'},
	{'name': 'namespace',						'default': "",											'description': 'Application namespace'},
	{'name': 'use_sim_time',                  	'default': True,                						'description': 'Use simulation time'},
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
      
	# Robot localization for odometry
	start_odom_ekf = Node(
		package='robot_localization',
		executable='ekf_node',
		name='ekf_node_odom',
		output='screen',
        namespace=launch_config['namespace'],
		parameters=[launch_config['robot_localization_file_path'], {'use_sim_time': launch_config['use_sim_time']}],
        remappings=[
                ('odometry/filtered', '/odom/filtered')
            ],)
      
	# Robot state publisher
	start_robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
        name='robot_state_publisher',
		parameters=[{'use_sim_time': launch_config['use_sim_time'],
                	'robot_description': ParameterValue(Command(['xacro ', launch_config['robot_model']]), value_type=str)}],
		arguments=[launch_config['robot_model']])
	
	# Joint state publisher
	start_joint_state_publisher = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
        parameters=[{'use_sim_time': launch_config['use_sim_time']}],)


	# Declare the launch options
	declerations = declare_configurable_parameters(configurable_parameters)
	for param in declerations:
		ld.add_action(param)

	# Add any actions
	ld.add_action(start_odom_ekf)
	ld.add_action(start_robot_state_publisher)
	ld.add_action(start_joint_state_publisher)

	return ld
