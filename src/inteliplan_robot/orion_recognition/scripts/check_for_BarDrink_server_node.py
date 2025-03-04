#!/usr/bin/env python3

import rospy
from orion_recognition.check_for_BarDrink_server import CheckForBarDrinkServer


def main():
    rospy.init_node('check_for_BarDrink_server_node')
    CheckForBarDrinkServer()
    rospy.spin()


if __name__ == '__main__':
    main()
