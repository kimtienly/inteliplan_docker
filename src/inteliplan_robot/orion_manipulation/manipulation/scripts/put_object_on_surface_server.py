#! /usr/bin/env python3
""" Action server for putting objects on a surface.
"""

import rospy
import tf
import hsrb_interface.geometry as geometry
import traceback

import orion_actions.msg as msg
from manipulation.manipulation_header import ManipulationAction
from manipulation.collision_mapping import CollisionWorld
from point_cloud_filtering.srv import DetectSurface

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PutObjectOnSurfaceAction(ManipulationAction):

    # How far above the surface to let objects drop. If too small, the object may collide
    # with the surface if the height of the object is slightly misjudged
    DROP_HEIGHT = 0.01

    def __init__(
        self,
        action_name,
        action_msg_type=msg.PutObjectOnSurfaceAction,
        use_collision_map=True,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(PutObjectOnSurfaceAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        rospy.loginfo("%s: Waiting for /detect_surface service..." % self._action_name)
        rospy.wait_for_service("/detect_surface")
        self.detect_surface_service = rospy.ServiceProxy(
            "/detect_surface", DetectSurface
        )
        rospy.loginfo("%s: Got /detect_surface service" % self._action_name)

        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PickUpObjectAction
        """

        _result = msg.PutObjectOnSurfaceResult()
        _result.result = False
        _result.failure_mode = msg.PutObjectOnSurfaceResult.SUCCESS

        goal_tf = goal_msg.goal_tf
        rospy.loginfo(
            "%s: Requested to put object on surface %s" % (self._action_name, goal_tf)
        )
        # Look at the goal - make sure that we get all of the necessary collision map
        rospy.loginfo("%s: Moving head to look at the location." % self._action_name)
        self.look_at_object(goal_tf)
        # rospy.sleep(0.5)

        # Attempt to find transform from camera frame to goal_tf
        (rgbd_goal_transform, _) = self.lookup_transform(
            self.RGBD_CAMERA_FRAME, goal_tf, timeout=rospy.Duration(5)
        )

        if rgbd_goal_transform is None:
            rospy.logerr("%s: Unable to find TF frame." % self._action_name)
            self.tts_say("I don't know the surface frame you want to put object down.")
            self.abandon_action()
            _result.failure_mode = msg.PutObjectOnSurfaceResult.TF_TIMEOUT
            return

        if self.handle_possible_preemption():
            return

        # Evaluate collision environment
        if self.use_collision_map:
            self.tts_say("Evaluating a collision-free path.", duration=1.0)
            collision_world = self.get_goal_cropped_collision_map(
                goal_tf, crop_dist_3d=0)
        else:
            collision_world = CollisionWorld.empty(self.whole_body)

        if self.handle_possible_preemption():
            return

        place_success = False

        object_half_height = goal_msg.object_half_height if goal_msg.object_half_height!=0 else 0.1;

        try:
            place_success = self.do_placement(
                goal_tf,
                collision_world,
                rgbd_goal_transform,
                goal_msg.abandon_action_if_no_plane_found,
                goal_msg.drop_object_by_metres,
                goal_msg.check_weight_grams,
                goal_msg.shelf_tf_ref,
                object_half_height=object_half_height
            )

        except Exception as e:
            rospy.logerr("%s: Encountered exception %s." % (self._action_name, str(e)))
            rospy.logerr(traceback.format_exc())
            self.abandon_action()

            _result.failure_mode = msg.PutObjectOnSurfaceResult.PLACING_EXCEPTION
            return

        if place_success:
            self.tts_say("Place successful.")
            # Now return to moving position
        else:
            self.tts_say("Failed to place")

        self.finish_position(collision_world)

        if place_success:
            rospy.loginfo("%s: Placing succeeded" % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo("%s: Placing failed" % self._action_name)
            _result.result = False

            _result.failure_mode = msg.PutObjectOnSurfaceResult.PLACING_FAILED
            self._as.set_aborted()

    def do_placement(
        self,
        goal_tf,
        collision_world,
        rgbd_goal_transform,
        abandon_action_if_no_plane_found,
        drop_by,
        check_weight_grams,
        shelf_tf_ref,
        object_half_height=0.05
    ):

        rospy.loginfo("%s: Running surface detection" % self._action_name)
        # Segment surface
        plane_transform = self.detect_plane_surface(rgbd_goal_transform)
        found_plane = plane_transform is not None

        if found_plane:
            rospy.loginfo("%s: Received a plane transform." % self._action_name)
            target_id = "placement_plane"
            self.publish_tf(plane_transform, self.RGBD_CAMERA_FRAME, target_id)

        elif not abandon_action_if_no_plane_found:
            rospy.loginfo(
                "%s: Did not receive a plane transform, continuing" % self._action_name
            )
            if shelf_tf_ref == "":
                rospy.loginfo("Considering plane at goal_tf")
                target_id = goal_tf
            else:
                rospy.loginfo("Considering plane at {}".format(shelf_tf_ref))
                target_id = "placement_plane"
                (shelf_goal_trans, _) = self.lookup_transform(goal_tf, shelf_tf_ref, timeout=rospy.Duration(0.5))
                shelf_goal_trans.translation.x = 0
                shelf_goal_trans.translation.y = 0
                self.publish_tf(shelf_goal_trans, goal_tf, target_id)
        else:
            rospy.loginfo(
                "%s: Did not receive a plane transform, stopping" % self._action_name
            )
            return False

        rel_placement_pose = self.get_relative_placement(drop_by=drop_by, object_half_height=object_half_height)
        base_target_pose = self.get_relative_effector_pose(
            target_id, relative=rel_placement_pose, publish_tf="goal_pose"
        )

        # Error checking in case we can't get a valid pose
        if base_target_pose is None:
            self.abandon_action()
            return False

        self.whole_body.end_effector_frame = self.HAND_FRAME

        with collision_world:
            self.whole_body.move_end_effector_pose(base_target_pose, self.BASE_FRAME)

            if check_weight_grams > 0.0:
                self.wrist_force.zero()

            # Let go of the object
            self.tts_say("Placing object.")
            rospy.loginfo("%s: Opening gripper." % (self._action_name))
            self.gripper.command(1.2)
            rospy.sleep(1.0)

            if check_weight_grams > 0.0:
                weight_difference_grams = self.wrist_force.get_absolute_grams_delta()
                rospy.loginfo("%s: Weight change is %i g." % (self._action_name, weight_difference_grams))

                if weight_difference_grams < check_weight_grams:
                    # Not enough weight change - shake the hand around
                    self.shake_gripper()

            # Move the gripper back a bit then return to go
            self.whole_body.angular_weight = 10;
            self.whole_body.linear_weight = 100;
            try:
                self.whole_body.move_end_effector_pose(
                    geometry.pose(z=-0.2), "hand_palm_link"
                )
            except:
                rospy.logwarn("Error when moving the arm backwards");

        return True

    def shake_gripper(self):
        self.whole_body.move_to_joint_positions({"wrist_roll_joint": -1.8})
        self.whole_body.move_to_joint_positions({"wrist_roll_joint": 1.8})
        self.whole_body.move_to_joint_positions({"wrist_roll_joint": 0})

    def get_relative_placement(self, object_half_height=0.05, drop_by=0.0):
        """
        Request the surface_detection node to detect the surface location.
        Args:
            object_half_height: how far the carried object extends below the gripper axis
                                (from hand-palm link). This should come from keeping track
                                of the height of the object when it is picked up, but at
                                the moment we don't have any component that does this.
        """
        return geometry.pose(
            z=-0.08, x=object_half_height + self.DROP_HEIGHT + max(drop_by, 0.0)
        )

    def detect_plane_surface(self, plane_search_transform_in_head_frame):
        """
        Request the surface_detection node to detect the surface location.
        Z component is used to find the plane axis: Z vector of transform should align
        with where the plane axis is expected to be.
        """
        try:
            res = self.detect_surface_service(
                plane_search_transform_in_head_frame,
                15.0,  # EPS plane search angle tolerance in degrees
                0.3,  # Box crop size to search for plane in. Axis aligned w/ head frame.
            )
            if res.success:
                return res.plane_axis
            else:
                return None

        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return None


if __name__ == "__main__":
    rospy.init_node("put_object_on_surface_server")
    server = PutObjectOnSurfaceAction(
        "put_object_on_surface", use_collision_map=True,
    )
    rospy.spin()
