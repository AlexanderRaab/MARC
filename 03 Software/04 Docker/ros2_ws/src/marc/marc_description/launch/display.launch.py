import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

''' Launch file for displaying the robot model with rviz2

'''

def generate_launch_description():
    # Setup paths and file names
	pkg_share = FindPackageShare(package='marc_description').find('marc_description')  
	default_model_path = os.path.join(pkg_share, 'src/robot/MARC.urdf.xacro')
	default_rviz_config_path = os.path.join(pkg_share, 'launch/urdf_config.rviz')
    
	# Configurations
	model = LaunchConfiguration('model')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    	# Launch arguments
	declare_model_path = DeclareLaunchArgument(name='model', default_value=default_model_path, 
    					description='Absolute path to robot urdf file')

	declare_rviz_config_file = DeclareLaunchArgument(name='rviz_config_file', default_value=default_rviz_config_path,
					description='Absolute path to rviz config file')
	
	# Subscribe to the joint states of the robot, and publish the 3D pose of each link.
	start_robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		parameters=[{'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)}],
		arguments=[default_model_path])
		
	start_joint_state_publisher = Node(
		package='joint_state_publisher',
        	executable='joint_state_publisher',
        	name='joint_state_publisher')

  	# Launch RViz
	start_rviz = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file])
			     	

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_model_path)
	ld.add_action(declare_rviz_config_file)

	# Add any actions
	ld.add_action(start_robot_state_publisher)
	ld.add_action(start_joint_state_publisher)
	ld.add_action(start_rviz)

	return ld
