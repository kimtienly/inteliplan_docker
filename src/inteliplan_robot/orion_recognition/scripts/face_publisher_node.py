#!/usr/bin/env python3

import rospy
import os
from orion_recognition.face_publisher_new import FacePublisher


def main():
    rospy.init_node('face_publisher_node')
    img_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    FacePublisher(img_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
