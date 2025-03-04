#!/usr/bin/env python3

import rospy
from orion_recognition.pose_tf_publisher import DetectionTFPublisher


def main():
    rospy.init_node('detection_tf_publisher_pose')
    DetectionTFPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
