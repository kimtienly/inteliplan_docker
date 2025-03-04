#!/usr/bin/env python3

import rospy
import os
from orion_recognition.bbox_publisher import BboxPublisher


def main():
    rospy.init_node('bbox_publisher_node')
    img_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    depth_topic = "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
    BboxPublisher(img_topic, depth_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
