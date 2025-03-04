#! /usr/bin/env python3
""" Client for put_object_on_surface action.
"""

import rospy
import sys
import actionlib
import orion_actions.msg as msg


def put_object_on_surface_client(goal_tf, shelf_tf_ref_):
    client = actionlib.SimpleActionClient("put_object_on_surface", msg.PutObjectOnSurfaceAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    # See PutObjectOnSurface.action to see description of these fields
    goal_msg = msg.PutObjectOnSurfaceGoal(goal_tf=goal_tf,
                                          abandon_action_if_no_plane_found=False,
                                          drop_object_by_metres=0.0,
                                          check_weight_grams=-1,
                                          shelf_tf_ref=shelf_tf_ref_)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("put_object_on_surface_client")
    
    
    if len(sys.argv) >= 2:
        goal_tf = sys.argv[1]
        try:
            shelf_tf_ref = sys.argv[2]
        except:
            shelf_tf_ref = None
    else:
        print("Failed to provide tf frame as argument - defaulting to surface_placement_l")
        goal_tf = "surface_placement_l"

    result = put_object_on_surface_client(goal_tf, shelf_tf_ref)
    print("Result:" + str(result.result))
