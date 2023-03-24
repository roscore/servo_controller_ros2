from  launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='ether_ros2',
			node_namespace='ether_ros2',
			node_executable='ethercat_multiexec',
			node_name='myether',
		), ])

#			prefix=['ethercat_grant'],
