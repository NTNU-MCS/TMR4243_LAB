#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickMapping

def joystick_force_basin_relative(
        joystick: sensor_msgs.msg.Joy,
        position: np.ndarray,
        mapping: JoystickMapping) -> np.ndarray:
    # Replace the following line
    u0, u1, u2, a1, a2 = 0.0, 0.0, 0.0, 0.0, 0.0
    tau0, tau1, tau2 = 0.0, 0.0, 0.0,

    eta = position
    #
    ## Write your code below
    #

    tau = np.array([[tau0, tau1, tau2, a1, a2]], dtype=float).T
    return tau