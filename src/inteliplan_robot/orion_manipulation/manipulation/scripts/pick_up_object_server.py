#! /usr/bin/env python3
""" Action server for picking up objects.
Key options:
    - use_grasp_synthesis:
        True:   Use gpd / gpg NN-based grasp pose synthesis.
        False:  Use hard-coded grasp synthesis (a few cm from target centre, horizontal).
    - use_collision_map:
        True:   Automatically avoid obstacles.
        False:  Do not avoid obstacles. ONLY FOR SUPERVISED TESTING WITH E-STOP.
"""

import numpy as np
import rospy
import actionlib
import tf
import tf2_ros;
import tf.transformations as T
import traceback
import hsrb_interface.geometry as geometry
import math;

import orion_actions.msg as msg
# import orion_actions.srv as srv
import geometry_msgs.msg;
from actionlib_msgs.msg import GoalStatus
from manipulation.manipulation_header import ManipulationAction
from manipulation.collision_mapping import CollisionWorld
from point_cloud_filtering.srv import SegmentObject
# from gpd.msg import GraspConfigList
from tmc_suction.msg import SuctionControlAction, SuctionControlGoal

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PickUpObjectAction(ManipulationAction):

    ACTION_SERVER_CONNECTION_TIMEOUT = 15.0  # Used for e.g. vacuum action server

    # How recently we must have seen an object to pick it up
    GOAL_OBJECT_TF_TIMEOUT = 100.0

    # How long we give grasp pose generation to generate grasps
    GRASP_POSE_GENERATION_TIMEOUT = 7.0

    SUCTION_TIMEOUT = rospy.Duration(20.0)  # Vacuum action timeout
    DEFAULT_GRASP_FORCE = 0.8
    # Relative to goal_tf, in coordinate system of hand_palm_link (x=up in neutral position)
    PREGRASP_POSE = geometry.pose(z=-0.08, ek=0)
    GRASP_POSE = geometry.pose(z=0.06)  # Relative to gripper
    LIFT_POSE = geometry.pose(x=0.03)  # Relative to gripper, to lift off surface
    BIN_PREGRASP_DISTANCE = 0.2 # Relative to the handle of the bin bag

    TF_PUBLISHED_NAME = "MANIP_GOAL_TF_PICKUP"

    def __init__(
        self,
        action_name,
        action_msg_type=msg.PickUpObjectAction,
        use_collision_map=True,
        use_grasp_synthesis=True,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(PickUpObjectAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        self.use_grasp_synthesis = use_grasp_synthesis
        print("Using grasp pose synthesis: ", self.use_grasp_synthesis);

        if self.use_grasp_synthesis:
            rospy.loginfo("%s: Grasp pose synthesis is enabled" % self._action_name)
            rospy.loginfo(
                "%s: Waiting for /segment_object service..." % self._action_name
            )
            rospy.wait_for_service("/segment_object")
            self.segment_object_service = rospy.ServiceProxy(
                "/segment_object", SegmentObject
            )
            rospy.loginfo("%s: Got /segment_object service" % self._action_name)

            self.grasps = None
            self.grasp_sub = rospy.Subscriber(
                "/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_callback
            )
        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

        print(dir(self.whole_body));

        self.transform_broadcaster = tf2_ros.StaticTransformBroadcaster();

    def _execute_cb(self, goal_msg:msg.PickUpObjectGoal):
        """
        Action server callback for PickUpObjectAction
        """
        _result = msg.PickUpObjectResult()
        _result.result = False
        _result.failure_mode = msg.PickUpObjectResult.SUCCESS

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to pick up tf %s" % (self._action_name, goal_tf))

        lookup_timeout = rospy.Duration(0);

        if goal_msg.publish_own_tf:
            query = srv.SOMQueryObjectsRequest();
            query.query.tf_name = goal_tf;
            rospy.wait_for_service('/som/objects/basic_query');
            object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', srv.SOMQueryObjects);
            result:srv.SOMQueryObjectsResponse = object_query_srv(query);
            result_of_interest:msg.SOMObject = result.returns[0];
            individual_tf = geometry_msgs.msg.TransformStamped();
            individual_tf.header.frame_id = "map";
            individual_tf.header.stamp = rospy.Time.now();
            individual_tf.child_frame_id = self.TF_PUBLISHED_NAME;
            individual_tf.transform.translation = result_of_interest.obj_position.position;
            individual_tf.transform.rotation.w = 1;

            tf_list = [individual_tf];
            self.transform_broadcaster.sendTransform(tf_list);

            goal_tf = self.TF_PUBLISHED_NAME;
            rospy.loginfo("Trying to pick up the tf {0}".format(goal_tf));

            lookup_timeout = rospy.Duration(5);
            pass;

        self.whole_body.angular_weight = 10;
        self.whole_body.linear_weight = 100;


        approach_axis = goal_msg.approach_axis
        if len(approach_axis) != 0 and len(approach_axis) != 3:
            rospy.logerr("Approach direction vector not given in correct format")
            self.abandon_action()
            return
        if len(approach_axis) == 0 or approach_axis == (0, 0, 0):
            approach_axis = None
        rospy.loginfo("Requested pick-up direction: {}".format("default" if approach_axis is None else str(approach_axis)))

        extend_distance = goal_msg.extend_distance
        is_bin_bag = goal_msg.is_bin_bag

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf, lookup_timeout);

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know the object you want picked up.", duration=2.0)

            _result.failure_mode = msg.PickUpObjectResult.TF_NOT_FOUND
            self.abandon_action(_result)
            return

        pregrasp_pose = self.PREGRASP_POSE
        grasp_pose = self.GRASP_POSE
        lift_pose = self.LIFT_POSE

        if extend_distance != 0:
            grasp_pose = geometry.pose(z=extend_distance, ek=0)

        if is_bin_bag:
            pregrasp_pose = geometry.pose(z=-extend_distance-self.BIN_PREGRASP_DISTANCE)
            grasp_pose = geometry.pose(z=self.BIN_PREGRASP_DISTANCE)
            lift_pose = geometry.pose(z=-self.BIN_PREGRASP_DISTANCE)


        # Look at the object - make sure that we get all of the necessary collision map
        rospy.loginfo("%s: Moving head to look at the object." % self._action_name)
        try:
            self.look_at_object(goal_tf)
        except Exception as e:
            rospy.logwarn("Tf error (probably)");
            print(e);
            _result.failure_mode = msg.PickUpObjectResult.TF_TIMEOUT;
            _result.result = False;
            self.abandon_action(_result);
            return;



        if self.handle_possible_preemption():
            return

        transform_age = (rospy.Time.now() - lookup_time).to_sec()
        if transform_age > self.GOAL_OBJECT_TF_TIMEOUT:
            rospy.logerr(
                "%s: Most recent published goal TF frame is too old (%.3g seconds old)."
                % (self._action_name, transform_age)
            )
            self.tts_say("I can't see the object you want picked up.", duration=2.0)

            _result.failure_mode = msg.PickUpObjectResult.TF_TIMEOUT
            self.abandon_action(_result);
            return

        if self.handle_possible_preemption():
            return

        obj_dist = self.calc_transform_distance(trans)
        rospy.loginfo(
            "%s: Distance to object is %s m from hand." % (self._action_name, obj_dist)
        )
        self.tts_say(
            'The object is "{:.2f}" metres away.'.format(obj_dist), duration=2.0
        )

        if self.handle_possible_preemption():
            return

        # Evaluate collision environment
        if self.use_collision_map:
            self.tts_say("Evaluating a collision-free path.", duration=1.0)
            collision_world = self.get_goal_cropped_collision_map(goal_tf)
        else:
            collision_world = CollisionWorld.empty(self.whole_body)

        if self.handle_possible_preemption():
            return

        grasp_success = self.grab_object(
            goal_tf, collision_world, pregrasp_pose, grasp_pose, lift_pose, approach_axis
        )

        if self.handle_possible_preemption():
            return

        if grasp_success:
            # self.tts_say("Object grasped successfully.")
            # Now return to moving position
            self.finish_position(collision_world)
        else:
            self.tts_say("Failed to pick up the object")

        if grasp_success:
            rospy.loginfo("%s: Grasping succeeded" % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo("%s: Grasping failed" % self._action_name)
            _result.result = False

            _result.failure_mode = msg.PickUpObjectResult.GRASPING_FAILED
            self._as.set_aborted(_result)

    def grab_object(
        self, goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose, chosen_lift_pose, approach_axis=None
    ):
        self.whole_body.end_effector_frame = self.HAND_FRAME

        rospy.loginfo("%s: Opening gripper." % (self._action_name))
        self.gripper.command(1.2)

        rospy.loginfo("Is using collision mapping: %s" % (self.use_collision_map))
        rospy.loginfo("Is using grasp synthesis: %s" % (self.use_grasp_synthesis))

        try:
            successfully_positioned = False

            if self.use_grasp_synthesis:
                successfully_positioned = (
                    self.position_end_effector_grasp_pose_synthesis(
                        goal_tf, collision_world, chosen_grasp_pose
                    )
                )

            if not successfully_positioned:
                rospy.loginfo("%s: Using fixed grasp pose." % self._action_name)
                successfully_positioned = self.position_end_effector_fixed_grasp(
                    goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose, approach_axis
                )

            if not successfully_positioned:
                return False

            # Specify the force to grasp
            self.tts_say("Grasping object.")
            self.gripper.apply_force(self.DEFAULT_GRASP_FORCE)
            rospy.loginfo("%s: Object grasped." % self._action_name)

            self.whole_body.move_end_effector_pose(
                chosen_lift_pose, self.whole_body.end_effector_frame
            )

            return True

        except Exception as e:
            rospy.logerr("%s: Encountered exception %s." % (self._action_name, str(e)))
            rospy.logerr(traceback.format_exc())
            # self.abandon_action()
            return False

    def position_end_effector_grasp_pose_synthesis(
        self, goal_tf, collision_world, chosen_grasp_pose
    ):
        """
        Move to a valid grasping position using grasp pose synthesis.
        """
        # Clear the previous grasp list
        self.grasps = None

        # We must ensure that the z-direction of the goal TF is in the plane axis (i.e.
        # upwards) so that plane detection will detect the surface the object is on.
        (base_to_goal_oriented, _) = self.lookup_transform(self.BASE_FRAME, goal_tf)
        if base_to_goal_oriented is None:
            rospy.logerr(
                "%s: Unable to transform from base to goal_tf." % (self._action_name)
            )
            return False
        base_to_goal_oriented.rotation.x = 0
        base_to_goal_oriented.rotation.y = 0
        base_to_goal_oriented.rotation.z = 0
        base_to_goal_oriented.rotation.w = 1
        self.publish_tf(base_to_goal_oriented, self.BASE_FRAME, "oriented_goal_tf")

        # Pass the correctly oriented transform into object segmentation service
        (head_obj_plane_transform, _) = self.lookup_transform(
            self.RGBD_CAMERA_FRAME, "oriented_goal_tf"
        )

        if head_obj_plane_transform is None:
            rospy.logerr(
                "%s: Unable to transform from base to oriented_goal_tf."
                % (self._action_name)
            )
            return False

        rospy.loginfo("%s: Calling segmentation." % (self._action_name))
        # Call segmentation
        self.tts_say("Calculating grasp options")
        seg_response = self.segment_grasp_target_object(head_obj_plane_transform)
        rospy.loginfo("%s: Finished calling segmentation." % (self._action_name))

        if not seg_response:
            # Segmentation internal failure
            return False

        rospy.loginfo("%s: Waiting for grasps." % (self._action_name))
        # Grasps are sorted in descending order by score
        # Include a timeout
        start_time = rospy.Time.now()
        while self.grasps is None:
            rospy.sleep(0.2)
            elapsed_secs = (rospy.Time.now() - start_time).to_sec()
            if elapsed_secs > self.GRASP_POSE_GENERATION_TIMEOUT:
                break

        if self.grasps is None:
            # gpg failed to return a list of grasp poses
            rospy.logerr(
                "%s: GPG failed to produce grasp poses." % (self._action_name)
            )
            return False

        rospy.loginfo(
            "%s: Got grasps list of length %i." % (self._action_name, len(self.grasps))
        )

        with collision_world:
            found_valid_grasp = False
            for idx, grasp in enumerate(self.grasps):
                rospy.loginfo("Grasping trial %s" % idx)
                grasp_pose = self.grasp_to_pose(self.grasps[idx])
                self.publish_goal_pose_tf(grasp_pose)

                try:
                    self.whole_body.move_end_effector_pose(
                        grasp_pose, self.RGBD_CAMERA_FRAME
                    )
                    found_valid_grasp = True
                except:
                    # Failed to find valid motion to this grasp pose - continue search
                    pass

        if not found_valid_grasp:
            return False

        # Robot should now have successfully moved to target generated grasp pose,
        # taking offset into account, and using collision avoidance if necessary.

        # Move to grasp pose without collision checking
        rospy.loginfo("%s: Moving to grasp." % (self._action_name))
        self.tts_say("Moving to grasp.")
        self.whole_body.move_end_effector_pose(
            chosen_grasp_pose, self.whole_body.end_effector_frame
        )
        rospy.loginfo("%s: Finished moving to grasp position." % (self._action_name))
        return True

    def position_end_effector_fixed_grasp(
        self, goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose, approach_axis=None
    ):
        """
        Move to a valid grasping position using a fixed approach from robot to target.
        """

        # Move to pregrasp
        rospy.loginfo("%s: Calculating grasp pose." % (self._action_name))

        base_target_pose = self.get_relative_effector_pose(
            goal_tf, relative=chosen_pregrasp_pose, publish_tf="goal_pose", approach_axis=approach_axis
        )

        # Error checking in case we can't get a valid pose
        if base_target_pose is None:
            self.abandon_action()
            return False

        rospy.loginfo("%s: Moving to pre-grasp position." % (self._action_name))
        self.tts_say("Moving to pre-grasp.")

        with collision_world:
            try:
                self.whole_body.joint_weights = {'arm_lift_joint':1}
                self.whole_body.move_end_effector_pose(base_target_pose, self.BASE_FRAME);
            except:
                BASE_ROTATION = math.pi/2;
                rospy.logwarn("Initial planning failed.");
                # print(dir(self.whole_body));
                self.whole_body.linear_weight = 50;
                self.whole_body.joint_weights = {'arm_lift_joint':0.1}
                # self.whole_body.angular_weight = 
                # self.whole_body.move_to_neutral();
                # self.whole_body.move_to_joint_positions({
                #     'arm_lift_joint':0.5,
                #     'arm_flex_joint':-100*math.pi/180,
                #     'head_pan_joint':0,
                #     'head_tilt_joint':-math.pi/6,
                #     'wrist_flex_joint':0
                #     });
                # self.omni_base.follow_trajectory(
                #     [geometry.pose(ek=BASE_ROTATION)],
                #     time_from_starts=[10],
                #     ref_frame_id='base_footprint');
                # self.omni_base.go_rel(0,0,BASE_ROTATION,10);
                self.look_at_object(goal_tf, rotate_to_face = True);

                rospy.loginfo("Recomputing");
                try:
                    rospy.loginfo("\tGetting relative effector pose.");
                    base_target_pose = self.get_relative_effector_pose(
                        goal_tf, relative=chosen_pregrasp_pose, publish_tf="goal_pose", approach_axis=approach_axis)
                    rospy.loginfo("\tMoving to pre grasp (again).");
                    self.whole_body.move_end_effector_pose(base_target_pose, self.BASE_FRAME);
                except:
                    return False;

        # Move to grasp pose without collision checking
        rospy.loginfo("%s: Moving to grasp." % (self._action_name))
        self.tts_say("Moving to grasp.")
        self.whole_body.move_end_effector_pose(
            chosen_grasp_pose, self.whole_body.end_effector_frame);
        return True

    def suck_object(self, goal_tf, collision_world):
        """
        Currently unused.
        """
        self.whole_body.end_effector_frame = self.SUCKER_FRAME

        rospy.loginfo("%s: Closing gripper." % self._action_name)
        self.gripper.command(0.1)

        rospy.loginfo("%s: Turning on on the suction..." % self._action_name)

        # Create action client to control suction
        suction_action = "/hsrb/suction_control"
        suction_control_client = actionlib.SimpleActionClient(
            suction_action, SuctionControlAction
        )

        # Wait for connection
        try:
            if not suction_control_client.wait_for_server(
                rospy.Duration(self.ACTION_SERVER_CONNECTION_TIMEOUT)
            ):
                raise Exception(suction_action + " does not exist")
        except Exception as e:
            rospy.logerr(e)

        # Send a goal to start suction
        rospy.loginfo(
            "%s: Suction server found. Activating suction..." % self._action_name
        )
        suction_on_goal = SuctionControlGoal()
        suction_on_goal.timeout = self._SUCTION_TIMEOUT
        suction_on_goal.suction_on.data = True

        if (
            suction_control_client.send_goal_and_wait(suction_on_goal)
            == GoalStatus.SUCCEEDED
        ):
            rospy.loginfo("Suction succeeded. Object picked up")
        else:
            rospy.loginfo("Suction failed")
            return

        with collision_world:
            self.whole_body.move_end_effector_pose(
                geometry.pose(z=0.05, ei=3.14), goal_tf
            )

        # Get close and grasp without collision checking
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.045), goal_tf)

    def grasp_callback(self, msg):
        self.grasps = msg.grasps

    def grasp_to_pose(self, grasp):
        """
        Convert a grasp pose synthesis-generated grasp into a pose for the actuator.
        """
        rospy.loginfo(
            "%s: Evaluating grasp with score:: %s"
            % (self._action_name, str(grasp.score))
        )

        # This gives the approach point correctly
        bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        hand_outer_diameter = 0.12
        hw = 0.5 * hand_outer_diameter
        finger_width = 0.01
        left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
        right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
        base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
        approach_center = base_center - 0.06 * approach

        approach_4 = np.array(
            [grasp.approach.x, grasp.approach.y, grasp.approach.z, approach_center[0]]
        )
        binormal_4 = np.array(
            [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]]
        )
        axis_4 = np.array(
            [grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]]
        )

        R = np.array([axis_4, -binormal_4, approach_4, [0, 0, 0, 1]])
        q = T.quaternion_conjugate(T.quaternion_from_matrix(R))

        return geometry.Pose(
            geometry.Vector3(
                approach_center[0], approach_center[1], approach_center[2]
            ),
            geometry.Quaternion(q[0], q[1], q[2], q[3]),
        )

    def segment_grasp_target_object(self, object_and_plane_transform_in_head_frame):
        """
        Request the /segment_object node to publish a segmented point cloud at a
        specified location in the head RGBD frame, containing the object we wish to
        generate grasp poses for.
        The grasp pose synthesis node will input this point cloud and publish a list of
        candidate poses.
        Z component is used to find the plane axis: Z vector of transform should align
        with where the plane axis is expected to be.
        """
        try:
            res = self.segment_object_service(
                object_and_plane_transform_in_head_frame,
                10.0,  # EPS plane search angle tolerance in degrees
                0.2,  # Box crop size to search for plane in. Axis aligned w/ head frame.
            )
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return False


if __name__ == "__main__":
    rospy.init_node("pick_up_object_server_node")
    server = PickUpObjectAction(
        "pick_up_object", use_collision_map=True, use_grasp_synthesis=False
    )
    rospy.spin()
