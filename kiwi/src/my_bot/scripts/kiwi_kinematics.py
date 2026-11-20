#!/usr/bin/env python3
"""Kiwi (3x omni) drive node.

/cmd_vel      -> inverse kinematics -> /wheel_velocity_controller/commands (rad/s)
/joint_states -> forward kinematics -> /odom + TF (odom -> base_link)

Wheel i sits at angle theta_i around base_link, distance L from the centre.
Positive wheel rotation drives the base along the tangent t_i = (-sin th, cos th)
(checked against the wheel rpy/axis in robot_core), so:

    w_i = (1/R) * ( -sin(th_i)*vx + cos(th_i)*vy + L*wz )
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class KiwiDrive(Node):
    def __init__(self):
        super().__init__('kiwi_drive')

        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('base_radius', 0.17)  # base_link origin -> wheel centre
        self.declare_parameter('wheel_joints',
                               ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint'])
        self.declare_parameter('wheel_angles_deg', [30.0, 150.0, 270.0])
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        R = self.get_parameter('wheel_radius').value
        L = self.get_parameter('base_radius').value
        self.joints = list(self.get_parameter('wheel_joints').value)
        angles = [math.radians(a) for a in self.get_parameter('wheel_angles_deg').value]
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # A maps body twist [vx, vy, wz] -> wheel speeds [rad/s]
        self.A = np.array([[-math.sin(a), math.cos(a), L] for a in angles]) / R
        self.A_inv = np.linalg.inv(self.A)

        # odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = None

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)

        self.get_logger().info('Kiwi drive node up: cmd_vel -> wheels, joint_states -> odom')

    # ---- inverse kinematics ----
    def on_cmd_vel(self, msg: Twist):
        w = self.A @ np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        out = Float64MultiArray()
        out.data = [float(v) for v in w]
        self.cmd_pub.publish(out)

    # ---- forward kinematics / odometry ----
    def on_joint_states(self, msg: JointState):
        vel = dict(zip(msg.name, msg.velocity))
        try:
            w = np.array([vel[j] for j in self.joints])
        except KeyError:
            return  # message doesn't contain our wheels (or no velocities yet)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0:
            return

        vx, vy, wz = self.A_inv @ w

        # integrate in the odom frame
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += wz * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = KiwiDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
