#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickMapping

def joystick_force_body_relative(
        joystick: sensor_msgs.msg.Joy,
        mapping: JoystickMapping) -> np.ndarray:
    # Replace the following line
    tau0, tau1, tau2 = 0.0, 0.0, 0.0
    #
    ## Write your code below
    #


    tau = np.array([[tau0, tau1, tau2]], dtype=float).T
    return tau