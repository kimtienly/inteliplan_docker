#!/usr/bin/env python3

import actionlib
import rospy
import orion_actions
# from orion_actions.msg import Detection, DetectionArray
from tmc_vision_msgs.msg import Detection, DetectionArray


class CheckForBarDrinkServer(object):
    def __init__(self):
        self.BarDrink = ['beer', 'chocolate_milk', 'coke', 'juice', 'lemonade', 'tea_bag']
        rospy.loginfo("Check for bar drink -- initialising check for object action server")
        self._check_for_BarDrink_as = actionlib.SimpleActionServer('check_for_BarDrink',
                                                                   orion_actions.msg.CheckForBarDrinkAction,
                                                                   execute_cb=self._check_for_bardrink_cb, auto_start=False)

        rospy.loginfo("Check for bar drink -- starting check for object action server")
        self._check_for_BarDrink_as.start()

        rospy.loginfo("Check for bar drink -- all initialised")

    def _check_for_bardrink_cb(self):
        rospy.loginfo("Check for bar drink -- executing check for bar drink callback function")
        if self._check_and_send_result_if_succeeded():
            rospy.loginfo("Check for bar drink -- callback done")
            return

        result = orion_actions.msg.CheckForBarDrinkActionResult()
        result.is_present = False
        result.drinks = []
        self._check_for_object_as.set_succeeded(result)
        rospy.loginfo("Check for bar drink -- callback done")

    def _is_object_in_detections(self, detections):
        rospy.loginfo("Check for bar drink -- check if bar drink above the bar")

        table_found = False
        for detection in detections.detections:
            if 'table' in detection.label.name:
                table_y = detection.y
                table_found = True
        if not table_found:
            rospy.loginfo("No table found.")
            return False, []

        bar_drinks = []
        for det in detections.detections:
            if det.label.name in self.BarDrink and det.y < table_y:
                bar_drinks.append(det.label.name)
        if len(bar_drinks) != 0: 
            return True, bar_drinks
        else:
            rospy.loginfo("No drinks available on the Bar.")
            return False, []
            
        rospy.loginfo("Bar is not detected.")
        return False, []

    def _check_and_send_result_if_succeeded(self):
        rospy.loginfo("Check for bar drink -- checking and sending result if succeeded")
        detections = rospy.wait_for_message('/vision/bbox_detections', DetectionArray, 5 * 60)
        rospy.loginfo("Check for bar drink -- detections received")

        is_present, bar_drinks = self._is_object_in_detections(detections)
        if is_present:
            result = orion_actions.msg.CheckForObjectActionResult()
            result.is_present = True
            result.drinks = bar_drinks
            self._check_for_object_as.set_succeeded(result)
            return True
        return False
