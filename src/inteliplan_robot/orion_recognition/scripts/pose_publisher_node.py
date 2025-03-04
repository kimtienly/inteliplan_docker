#!/usr/bin/env python3

import rospy
import os
from orion_recognition.pose_publisher import PosePublisher


def main():
    rospy.init_node('pose_publisher_node')
    img_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    PosePublisher(img_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
