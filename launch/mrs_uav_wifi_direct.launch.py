"""Start topic sharing with the UAV name and clock settings from the environment."""

import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	"""Read UAV_NAME, falling back to the system hostname, and load the YAML settings."""
	hostname = os.getenv("UAV_NAME") or socket.gethostname()
	return LaunchDescription(
		[
			DeclareLaunchArgument(
				"config_path",
				description="Path to the topic-sharing and Wi-Fi configuration YAML file.",
				default_value=PathJoinSubstitution(
					[
						FindPackageShare("mrs_uav_wifi_direct"),
						"config",
						"default.yaml",
					]
				),
			),
			DeclareLaunchArgument(
				"use_sim_time",
				default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
				description="Use the ROS simulation clock; transport rate timers use wall time.",
			),
			Node(
				package="mrs_uav_wifi_direct",
				executable="mrs_uav_wifi_direct_node",
				output="screen",
				namespace=f"/{hostname}/wifi",
				parameters=[
					{
						"config_path": LaunchConfiguration("config_path"),
						"hostname": hostname,
						"use_sim_time": LaunchConfiguration("use_sim_time"),
					}
				],
			),
		]
	)
