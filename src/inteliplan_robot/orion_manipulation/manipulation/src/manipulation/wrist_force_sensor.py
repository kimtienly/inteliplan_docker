#!/usr/bin/env python3
"""Get the weight of a grasped object. No guarantees on how accurate this is.
To integrate with all grasping servers. Adapted code (ForceSensorCapture) from Toyota
Motor Corporation. Adapted from work by Mark Finean.
"""

import math
import rospy

from geometry_msgs.msg import WrenchStamped


def root_squared_difference(pre_data_list, post_data_list):
    if len(pre_data_list) != len(post_data_list):
        raise ValueError("Argument lists differ in length")

    # Calculate square sum of difference
    square_sums = sum(
        [math.pow(b - a, 2) for (a, b) in zip(pre_data_list, post_data_list)]
    )

    return math.sqrt(square_sums)


class WristForceSensorCapture(object):
    """Subscribe and hold force sensor data
    - Copyright (C) 2016 Toyota Motor Corporation
    """

    CONNECTION_TIMEOUT = 5.0

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        self.zeroed_force_list = None

        # Subscribe force torque sensor data
        ft_sensor_topic = "/hsrb/wrist_wrench/raw"
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__hand_force_sensor_cb
        )

        rospy.wait_for_message(
            ft_sensor_topic, WrenchStamped, timeout=self.CONNECTION_TIMEOUT
        )

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __hand_force_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

    def zero(self):
        self.zeroed_force_list = self.get_current_force()

    def get_absolute_grams_delta(self):
        """
        NOTE: always positive (RMS). Same values returned after having picked up or put
        down an object with a given weight.
        """
        if self.zeroed_force_list is None:
            raise Exception("Force capture has not been zeroed")

        force_vector = self.get_current_force()

        force_difference = root_squared_difference(self.zeroed_force_list, force_vector)
        weight_difference = round(force_difference / 9.81 * 1000, 1)

        return weight_difference
