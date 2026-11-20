import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    controllers = os.path.join(pkg_path, 'config', 'kiwi_controllers_real.yaml')

    serial_port = LaunchConfiguration('serial_port')
    enc_counts_per_rev = LaunchConfiguration('enc_counts_per_rev')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_radius = LaunchConfiguration('base_radius')

    # URDF for the REAL robot (use_sim:=false -> ros2_control.xacro instead of the Gazebo plugin)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_sim:=false',
                 ' serial_port:=', serial_port,
                 ' enc_counts_per_rev:=', enc_counts_per_rev]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}])

    # The controller_manager (in simulation Gazebo creates this; on the robot we start it ourselves)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers])

    joint_broad = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster'], output='screen')
    wheel_ctrl = Node(package='controller_manager', executable='spawner',
                      arguments=['wheel_velocity_controller'], output='screen')

    # Opening the serial port resets the ESP32 and the hardware interface waits ~2.5 s for it to boot,
    # so give the controller_manager a head start before spawning controllers.
    start_joint_broad = TimerAction(period=4.0, actions=[joint_broad])
    start_wheel_ctrl = RegisterEventHandler(OnProcessExit(
        target_action=joint_broad, on_exit=[wheel_ctrl]))

    kiwi_node = Node(
        package='my_bot', executable='kiwi_kinematics.py', output='screen',
        parameters=[{
            'use_sim_time': False,
            'wheel_radius': ParameterValue(wheel_radius, value_type=float),
            'base_radius': ParameterValue(base_radius, value_type=float),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial device'),
        DeclareLaunchArgument('enc_counts_per_rev', default_value='1920',
                              description='Encoder counts per WHEEL revolution (as the firmware counts them)'),
        DeclareLaunchArgument('wheel_radius', default_value='0.04',
                              description='Wheel radius in metres (measure your real wheel)'),
        DeclareLaunchArgument('base_radius', default_value='0.17',
                              description='Distance from robot centre to wheel centre in metres'),
        rsp,
        control_node,
        start_joint_broad,
        start_wheel_ctrl,
        kiwi_node,
    ])
