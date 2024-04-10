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
# Maintainer: Emir Cem Gezer, Petter Hangerhageen
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com, petthang@stud.ntnu.no
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import rcl_interfaces.msg
import tmr4243_interfaces.msg
import geometry_msgs.msg

from template_controller.PID_controller import PID_controller
from template_controller.PD_FF_controller import PD_FF_controller
from template_controller.backstepping_controller import backstepping_controller

class Controller(rclpy.node.Node):
    TASK_PD_FF_CONTROLLER = 'PD_FF_controller'
    TASK_PID_CONTROLLER = 'PID_controller'
    TASK_BACKSTEPPING_CONTROLLER = 'backstepping_controller'
    TASKS = [TASK_PD_FF_CONTROLLER, TASK_PID_CONTROLLER, TASK_BACKSTEPPING_CONTROLLER]

    def __init__(self):
        super().__init__("tmr4243_controller")

        self.pubs = {}
        self.subs = {}

        self.subs["reference"] = self.create_subscription(
            tmr4243_interfaces.msg.Reference, '/CSEI/control/reference', self.received_reference, 10)

        self.subs['observer'] = self.create_subscription(
            tmr4243_interfaces.msg.Observer, '/CSEI/control/observer', self.received_observer ,10)

        self.pubs["generalized_forces"] = self.create_publisher(
             geometry_msgs.msg.Wrench, '/CSEI/generalized_forces', 1)

        self.p_gain = 1.0
        self.declare_parameter(
            "p_gain",
            self.p_gain,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Proportional gain",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )
        self.i_gain = 0.0
        self.declare_parameter(
            "i_gain",
            self.i_gain,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Integral gain",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )
        self.d_gain = 0.0
        self.declare_parameter(
            "d_gain",
            self.d_gain,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Derivative gain",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )
        self.k1_gain = 1.0
        self.declare_parameter(
            "k1_gain",
            self.k1_gain,
            rcl_interfaces.msg.ParameterDescriptor(
                description="K1 gain",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )
        self.k2_gain = 1.0
        self.declare_parameter(
            "k2_gain",
            1.0,
            rcl_interfaces.msg.ParameterDescriptor(
                description="K2 gain",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )

        self.task  = Controller.TASK_PD_FF_CONTROLLER
        self.declare_parameter(
            'task',
            self.control_task,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Task",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints=f"Allowed values: {' '.join(Controller.TASKS)}"
            )
        )
        self.current_controller.value

        self.last_reference = None
        self.last_observation = None

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        controller_period = 0.1 # seconds
        self.controller_timer = self.create_timer(controller_period, self.controller_callback)


    def timer_callback(self):

        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.p_gain = self.get_parameter("p_gain").get_parameter_value().double_value
        self.i_gain = self.get_parameter("i_gain").get_parameter_value().double_value
        self.d_gain = self.get_parameter("d_gain").get_parameter_value().double_value
        self.k1_gain = self.get_parameter("k1_gain").get_parameter_value().double_value
        self.k2_gain = self.get_parameter("k2_gain").get_parameter_value().double_value

        self.get_logger().info(f"Parameter task: {self.control_task}", throttle_duration_sec=1.0)


    def controller_callback(self):

        if self.last_observation == None:
            return

        if Controller.TASK_PD_FF_CONTROLLER in self.task:
            tau = PD_FF_controller(
                self.last_observation,
                self.last_reference,
                self.p_gain,
                self.d_gain
            )
        elif Controller.TASK_PID_CONTROLLER  in self.task:
            tau = PID_controller(
                self.last_observation,
                self.last_reference,
                self.p_gain,
                self.i_gain,
                self.d_gain
            )
        elif Controller.TASK_BACKSTEPPING_CONTROLLER in self.task:
            tau = backstepping_controller(
                self.last_observation,
                self.last_reference,
                self.k1_gain,
                self.k2_gain
            )
        else:
            tau = [0.0, 0.0, 0.0]

        if len(tau) != 3:
            self.get_logger().warn(f"tau has length of {len(tau)} but it should be 3: tau := [Fx, Fy, Mz]", throttle_duration_sec=1.0)
            return

        f = geometry_msgs.msg.Wrench()
        f.force.x = tau[0]
        f.force.y = tau[1]
        f.torque.z = tau[2]
        self.pubs["generalized_forces"].publish(f)

    def received_reference(self, msg):
        self.last_reference = msg

    def received_observer(self, msg):
        self.last_observation = msg


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

