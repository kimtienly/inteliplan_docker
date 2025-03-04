#!/usr/bin/env python3

import rospy
from orion_recognition.check_for_object_server import CheckForObjectServer


def main():
    rospy.init_node('check_for_object_server_node')
    CheckForObjectServer()
    rospy.spin()


if __name__ == '__main__':
    main()
