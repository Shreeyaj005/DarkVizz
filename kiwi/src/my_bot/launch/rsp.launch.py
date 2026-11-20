import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # gazebo_ros2_control (Humble) passes the whole URDF to rcl as a
    # "--param robot_description:=<xml>" override, which rcl parses as YAML.
    # Any ": " or " #" inside the text (e.g. in an XML comment) breaks that parse and the
    # controller_manager is never created. Comments are useless to the robot, so drop them.
    robot_description_config = re.sub(r'<!--.*?-->', '', robot_description_config, flags=re.S)

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])