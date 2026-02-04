#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Maintainer: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import math
import numpy as np

import rclpy
import rclpy.node

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from pyquaternion import Quaternion as quat


class UtilityNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('tmr4243_utility')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.pubs["tunnel"] = self.create_publisher(
            geometry_msgs.msg.Wrench, 'thruster/tunnel/command', 1)
        self.pubs["port"] = self.create_publisher(
            geometry_msgs.msg.Wrench, 'thruster/port/command', 1)
        self.pubs["starboard"] = self.create_publisher(
            geometry_msgs.msg.Wrench, 'thruster/starboard/command', 1)

        self.pubs["eta"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', 1)
        self.pubs["nu"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/nu', 1)
        self.pubs["tau"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/tau', 1)
        self.subs["u_cmd"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/u', self.u_command_callback, 10)

        self.subs["odometry"] = self.create_subscription(
            nav_msgs.msg.Odometry, 'measurement/odom', self.odometry_callback, 10)

        self.tau = np.zeros(3)
        self.eta = np.zeros(3)
        self.nu = np.zeros(3)

    def u_command_callback(self, msg: std_msgs.msg.Float32MultiArray):

        # Check if the message is of correct size
        if len(msg.data) != 5:
            self.get_logger().warn(
                f"ill sized u_cmd array with length {len(msg.data)}: {msg.data}")
            self.get_logger().info(f"{msg}")
            self.tau = np.zeros(3)
            return

        # Extract the data
        u0, u1, u2, a1, a2 = msg.data

        # Create the messages and publish
        f = geometry_msgs.msg.Wrench()
        f.force.x = u0
        self.pubs['tunnel'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u1 * math.cos(a1)
        f.force.y = u1 * math.sin(a1)
        self.pubs['port'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u2 * math.cos(a2)
        f.force.y = u2 * math.sin(a2)
        self.pubs['starboard'].publish(f)

        B = np.array([
            [0, np.cos(a1), np.cos(a2)],
            [1, np.sin(a1), np.sin(a2)],
            [0.3875, 0.0550 * np.cos(a1) - 0.4574 * np.sin(a1), -
             0.4574 * np.sin(a2) - 0.0550 * np.cos(a2)]
        ])

        np.clip(u0, -1.0, 1.0)
        np.clip(u1, -2.0, 2.0)
        np.clip(u2, -2.0, 2.0)

        self.tau = B @ np.array([[u0], [u1], [u2]])

        self.pubs['tau'].publish(std_msgs.msg.Float32MultiArray(data=list(self.tau.flatten())))

    def odometry_callback(self, msg: nav_msgs.msg.Odometry):
        self.nu[0] = msg.twist.twist.linear.x
        self.nu[1] = msg.twist.twist.linear.y
        self.nu[2] = msg.twist.twist.angular.z

        self.eta[0] = msg.pose.pose.position.x
        self.eta[1] = msg.pose.pose.position.y
        _, _, self.eta[2] = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.pubs['nu'].publish(std_msgs.msg.Float32MultiArray(data=list(self.nu.flatten())))
        self.pubs['eta'].publish(std_msgs.msg.Float32MultiArray(data=list(self.eta.flatten())))


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = UtilityNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
