#!/usr/bin/env python3

import rospy
from orion_recognition.detection_tf_publisher import DetectionTFPublisher


def main():
    rospy.init_node('detection_tf_publisher')
    DetectionTFPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
