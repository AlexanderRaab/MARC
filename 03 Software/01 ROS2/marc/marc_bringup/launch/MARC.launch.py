import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#########################################################################
###
### Main launch file for all MARC applications
###
#########################################################################



# Initialize default paths
pkg_share_bringup = get_package_share_directory('marc_bringup')
pkg_share_description = get_package_share_directory('marc_description')
pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

default_device_config_path = os.path.join(pkg_share_bringup, 'config', 'device_config.yaml')
default_robot_model_path = os.path.join(pkg_share_description, 'src', 'robot', 'MARC.urdf.xacro')
default_world_file_path = os.path.join(pkg_share_bringup, 'src', 'worlds', 'mt_og5.world')
default_rviz_config_path = os.path.join(pkg_share_bringup, 'config', 'default.rviz')


# Setup parameter confinguration
configurable_parameters = [
    # Paths and files
    {'name': 'robot_model',                   	'default': default_robot_model_path,         			'description': 'Robot model file'},
    {'name': 'world_file',                    	'default': default_world_file_path,         			'description': 'Gazebo world file'},
    {'name': 'rviz_config',                   	'default': default_rviz_config_path,         			'description': 'rviz config file'},
    {'name': 'namespace',						'default': "",											'description': 'Application namespace'},
	
	# Operation modes
    {'name': 'visual',                   		'default': True,                						'description': 'Use visualization'},
    {'name': 'simulation',                    	'default': True,                						'description': 'Use Gazbo simulation'},
    
	# Simulation
	{'name': 'use_sim_time',                  	'default': True,                						'description': 'Use simulation time'},
    
    # Physical components
    {'name': 'use_plattform',					'default': True,										'description': 'Activate the differential drive plattform'},
    {'name': 'use_lidar',						'default': True,										'description': 'Activate the main lidar scanner'},
    {'name': 'use_camera',						'default': True,										'description': 'Activate the main camera'},
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

	# --- Simulation related components
	start_simulation = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_share_bringup, 'launch', 'simulation_components.launch.py')),
		condition=IfCondition(launch_config['simulation']),
		launch_arguments={
				'namespace': launch_config['namespace']}.items())
    
	# --- Physical system components
	start_physical = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_share_bringup, 'launch', 'physical_components.launch.py')),
		condition=UnlessCondition(launch_config['simulation']),
		launch_arguments={
				'use_plattform': launch_config['use_plattform'],
                'use_lidar': launch_config['use_lidar'],
                'use_camera': launch_config['use_camera'],
                'base_device': "/dev/ttyACM0",
                'base_baudrate': "115200",
                'lidar_device': "/dev/ttyUSB0",
                'camera_device': "/dev/video0",
                'camera_framerate': "10.",
				'namespace': launch_config['namespace'],
                'test': "test"}.items())
      
	
	# --- Core system components
	start_core = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_share_bringup, 'launch', 'core_components.launch.py')),
		launch_arguments={
				'use_sim_time': launch_config['use_sim_time'],
				'namespace': launch_config['namespace']}.items())


  	# Launch RViz
	start_rviz = Node(
		condition=IfCondition(launch_config['visual']),
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', launch_config['rviz_config']])
			     		

	# Declare the launch options
	declerations = declare_configurable_parameters(configurable_parameters)
	for param in declerations:
		ld.add_action(param)

	# Add any actions
	ld.add_action(start_core)
	ld.add_action(start_simulation)
	ld.add_action(start_physical)
      
	ld.add_action(TimerAction(period=1., actions=[start_rviz]))

	return ld
