#! /usr/bin/env python3
""" Client for pick_up_object action.
"""

import rospy
import sys
import argparse
import actionlib
import orion_actions.msg as msg


def pick_up_object_client(goal_tf, approach_axis=None, extend_distance=0, is_bin_bag=False):
    client = actionlib.SimpleActionClient("pick_up_object", msg.PickUpObjectAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.PickUpObjectGoal(goal_tf=goal_tf, approach_axis=approach_axis, extend_distance=extend_distance, is_bin_bag=is_bin_bag)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()

def arg_parser():
    parser = argparse.ArgumentParser(description ='sort some integers.')

    parser.add_argument('goal_tf', type=str, nargs='?', default="")
    parser.add_argument('-a', '--approach_axis', type=str, default=None)
    parser.add_argument('-e', '--extend_distance', type=float, default=0)
    parser.add_argument('-b', '--bin_bag', action='store_true')

    return parser

def str_to_tuple(txt):
    assert txt[0] == '(' and txt[-1] == ')'
    split_txt = txt[1: -1].split(',')

    return tuple([float(item.strip()) for item in split_txt])

if __name__ == "__main__":
    rospy.init_node("pick_up_object_client")

    args = arg_parser().parse_args()

    goal_tf = args.goal_tf.strip()
    approach_axis = args.approach_axis
    extend_distance = args.extend_distance
    is_bin_bag = args.bin_bag

    if is_bin_bag:
        approach_axis = "(0,0,-1)"

    if approach_axis is not None:
        approach_axis = str_to_tuple(approach_axis)

    if goal_tf == "":
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    if extend_distance != 0 and approach_axis is None:
        print("Extension Direction should be specified through approach_direction argument")
        print("extend_distance not used")

    result = pick_up_object_client(goal_tf, approach_axis, extend_distance, is_bin_bag)
    print("Result:" + str(result.result))
