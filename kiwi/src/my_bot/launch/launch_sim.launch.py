import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')

    # Spawners must NOT start until the robot exists in Gazebo, because the
    # controller_manager is created by the gazebo_ros2_control plugin at spawn time.
    joint_broad = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster'], output='screen')
    wheel_ctrl = Node(package='controller_manager', executable='spawner',
                      arguments=['wheel_velocity_controller'], output='screen')

    start_joint_broad = RegisterEventHandler(OnProcessExit(
        target_action=spawn_entity,
        on_exit=[TimerAction(period=3.0, actions=[joint_broad])]))
    start_wheel_ctrl = RegisterEventHandler(OnProcessExit(
        target_action=joint_broad,
        on_exit=[wheel_ctrl]))

    kiwi_node = Node(package='my_bot', executable='kiwi_kinematics.py',
                     output='screen', parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        start_joint_broad,
        start_wheel_ctrl,
        kiwi_node,
    ])