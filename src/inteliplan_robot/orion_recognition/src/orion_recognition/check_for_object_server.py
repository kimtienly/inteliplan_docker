#!/usr/bin/env python3

import actionlib
import rospy
import orion_actions
from orion_actions.msg import Detection, DetectionArray


class CheckForObjectServer(object):
    def __init__(self):
        rospy.loginfo("Check for object -- initialising check for object action server")
        self._check_for_object_as = actionlib.SimpleActionServer('check_for_object',
                                                                 orion_actions.msg.CheckForObjectAction,
                                                                 execute_cb=self._check_for_object_cb, auto_start=False)

        rospy.loginfo("Check for object -- starting check for object action server")
        self._check_for_object_as.start()

        rospy.loginfo("Check for object -- all initialised")

    def _check_for_object_cb(self, goal):
        rospy.loginfo("Check for object -- executing check for object callback function")
        if self._check_and_send_result_if_succeeded(goal):
            rospy.loginfo("Check for object -- callback done")
            return

        result = orion_actions.msg.CheckForObjectActionResult()
        result.is_present = False
        self._check_for_object_as.set_succeeded(result)
        rospy.loginfo("Check for object -- callback done")

    def _is_object_in_detection(self, goal, detections):
        rospy.loginfo("Check for object -- check if object in detections")
        if goal is None or goal.object_name is None:
            return False

        for detection in detections.detections:
            if goal.object_name in detection.label.name:
                return True
        return False

    def _check_and_send_result_if_succeeded(self, goal):
        rospy.loginfo("Check for object -- checking and sending result if succeeded")
        detections = rospy.wait_for_message('/vision/bbox_detections', DetectionArray, 5 * 60)
        rospy.loginfo("Check for object -- detections received")

        is_present = self._is_object_in_detections(goal, detections)
        if is_present:
            result = orion_actions.msg.CheckForObjectActionResult()
            result.is_present = True
            self._check_for_object_as.set_succeeded(result)
            return True
        return False
