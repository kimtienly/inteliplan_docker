#!/usr/bin/env python3

import rospy
from orion_recognition.pointing_server import PointingServer


def main():
    rospy.init_node('pointing_server_node')
    PointingServer()
    rospy.spin()


if __name__ == '__main__':
    main()
