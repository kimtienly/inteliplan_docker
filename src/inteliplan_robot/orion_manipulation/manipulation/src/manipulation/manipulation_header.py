#! /usr/bin/env python3
""" Common manipulation action server functionality.
"""

from functools import partial
import rospy
import actionlib
import numpy as np
import math
import tf2_ros
import hsrb_interface
import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Transform, TransformStamped, Point
from manipulation.msg import BoundingBox
from manipulation.wrist_force_sensor import WristForceSensorCapture
from manipulation.collision_mapping import CollisionMapper
import tf.transformations

from hsrb_interface import robot as _robot

_robot.enable_interactive()


class ManipulationAction(object):

    # Robot default parameters
    DEFAULT_BODY_PLANNING_TIMEOUT = 4.0  # Robot planning timeout - default is 10s
    DEFAULT_BODY_TF_TIMEOUT = 5.0  # Robot TF timeout - default is 5s

    # Frame name constants
    HAND_FRAME = "hand_palm_link"
    SUCKER_FRAME = "hand_l_finger_vacuum_frame"
    RGBD_CAMERA_FRAME = "head_rgbd_sensor_rgb_frame"
    ODOM_FRAME = "odom"
    MAP_FRAME = "map"
    BASE_FRAME = "base_link"
    BASE_FOOTPRINT_FRAME = "base_footprint"

    # Whether to finally return to the map position the manipulation action was called at
    RETURN_TO_START_AFTER_ACTION = True
    RETURN_TO_START_GAZE_AFTER_ACTION = True

    # Kinematic parameters
    MAX_HEIGHT_ARM_LIFT_JOINT = 0.69
    # every 2cm the arm lift join lifts, the RGBD camera moves up by 1cm
    H_ARM_RGBD_SCALE_FACTOR = 2
    RGBD_MIN_HEIGHT = 0.967
    RGBD_MAX_HEIGHT = 1.312
    MIN_ANGLE_ARM_FLEX_HEIGHT = -2.6

    # Parameters estimated from RViz / robot
    # How far the arm swings when switching it from 0 to MIN_ANGLE_ARM_FLEX_HEIGHT radians
    ARM_SWING_DIST = 0.3  # metres
    MAX_HEIGHT_PLANE_VISIBLE = 3.0  # metres
    # Should aim to look down from about 10cm above
    LOOK_ANGLE_OFFSET = 0.1  # metres
    # How far up can ARM LIGHT JOINT go without the hand blocking the RGBD camera?
    MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION = 0.25  # metres

    def __init__(
        self,
        action_name,
        action_msg_type,
        use_collision_map=True,
        tts_narrate=True,
        prevent_motion=False,
    ):
        """
        Base class for manipulation actions.
        Args:
            action_name: The name of the action server
            action_msg_type: ROS message type for the action
            use_collision_map: Whether to use dynamic collision mapping to avoid obstacles
            tts_narrate: Whether to narrate what the robot is doing out loud with
                text-to-speech
            prevent_motion: forbid carrying out arm or base motion.
        """

        self._action_name = action_name
        self._action_msg_type = action_msg_type
        self.use_collision_map = use_collision_map

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get("whole_body")
        self.omni_base = self.robot.try_get("omni_base")

        # Override some motion functions if necessary
        self.prevent_motion = prevent_motion
        if self.prevent_motion:
            rospy.loginfo(
                "%s: Overriding base/arm motion functions." % self._action_name
            )
            self.whole_body.move_end_effector_pose = partial(
                self.override, "self.whole_body.move_end_effector_pose"
            )
            self.whole_body.move_to_go = partial(
                self.override, "self.whole_body.move_to_go"
            )
            self.whole_body.move_to_neutral = partial(
                self.override, "self.whole_body.move_to_neutral"
            )
            self.omni_base.go_rel = partial(self.override, "self.omni_base.go_rel")

        self.collision_world = None
        self.gripper = self.robot.try_get("gripper")
        self.whole_body.end_effector_frame = self.HAND_FRAME
        self.whole_body.looking_hand_constraint = True
        self.tts_narrate = tts_narrate
        if self.tts_narrate:
            self.tts = self.robot.try_get("default_tts")
            self.tts.language = self.tts.ENGLISH

        self.whole_body.planning_timeout = self.DEFAULT_BODY_PLANNING_TIMEOUT
        self.whole_body.tf_timeout = self.DEFAULT_BODY_TF_TIMEOUT

        # ORIon components
        self.collision_mapper = (
            CollisionMapper(self.robot) if use_collision_map else None
        )
        rospy.loginfo("%s: Waiting for wrist force sensor." % self._action_name)
        self.wrist_force = WristForceSensorCapture()
        rospy.loginfo("%s: Connected to wrist force sensor." % self._action_name)

        # TF defaults to buffering 10 seconds of transforms. Choose 60 second buffer.
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration.from_sec(60.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # All manipulation actions can publish a "goal" tf
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        if self._action_msg_type is not None:
            self._as = actionlib.SimpleActionServer(
                self._action_name,
                self._action_msg_type,
                execute_cb=self.execute_cb,
                auto_start=False,
            )
            self._as.start()
            rospy.loginfo("%s: Action server started." % self._action_name)

    def override(self, name, *args, **kwargs):
        """
        Used to override HSRB interface functions.
        """
        argstr = [str(a) for a in args]
        if len(kwargs) == 0:
            func = name + "(" + ", ".join(argstr) + ")"
        else:
            kwarg_strs = ["{}={}".format(k, v) for k, v in kwargs.items()]
            func = name + "(" + ", ".join(argstr) + ", " + ", ".join(kwarg_strs) + ")"
        rospy.loginfo("%s: Redirected %s" % (self._action_name, func))
        return True

    def execute_cb(self, goal_msg):
        """
        Do any bookkeeping here e.g. store pre-action map pose.
        """
        rospy.loginfo("%s: Received goal: %s" % (self._action_name, goal_msg))

        action_start_pose = self.omni_base.pose
        rospy.loginfo(
            "%s: Stored action start pose: %s" % (self._action_name, action_start_pose)
        )

        action_start_joint_poses = self.whole_body.joint_positions

        retval = self._execute_cb(goal_msg)

        if self.RETURN_TO_START_AFTER_ACTION:
            rospy.loginfo(
                "%s: Returning to start pose: %s"
                % (self._action_name, action_start_pose)
            )
            self.omni_base.go_abs(
                x=action_start_pose[0],
                y=action_start_pose[1],
                yaw=action_start_pose[2],
                timeout=20.0,
            )

        if self.RETURN_TO_START_GAZE_AFTER_ACTION:
            rospy.loginfo(
                "%s: Returning to start gaze: %s"
                % (
                    self._action_name,
                    str(
                        [
                            action_start_joint_poses["head_pan_joint"],
                            action_start_joint_poses["head_tilt_joint"],
                        ]
                    ),
                )
            )
            self.whole_body.move_to_joint_positions(
                {
                    "head_pan_joint": action_start_joint_poses["head_pan_joint"],
                    "head_tilt_joint": action_start_joint_poses["head_tilt_joint"],
                }
            )

        return retval

    def abandon_action(self, result=None):
        """
        Abandon manipulation action: move robot to go position and abort action server.
        """
        rospy.loginfo("%s: Aborted. Moving to go and exiting." % self._action_name)
        self.whole_body.move_to_go()
        if result==None:
            self._as.set_aborted()
        else:
            self._as.set_aborted(result);

    def preempt_action(self):
        """
        Preempt manipulation action: move robot to go position and preempt action server.
        """
        rospy.loginfo("%s: Preempted. Moving to go and exiting." % self._action_name)
        self.tts_say("I was preempted. Moving to go.")
        self.whole_body.move_to_go()
        self._as.set_preempted()

    def handle_possible_preemption(self):
        """
        Placed to enable preemption.
        """
        if self._as.is_preempt_requested():
            self.preempt_action()
            return True
        return False

    def publish_goal_pose_tf(
        self, p, source_frame="base_frame", goal_pose_name="goal_pose"
    ):
        """
        Publish a tf to the goal pose, in odom (or specified) frame
        Args:
            p: goal pose
        """
        t = Transform()
        t.translation.x = p.pos.x
        t.translation.y = p.pos.y
        t.translation.z = p.pos.z
        t.rotation.x = p.ori.x
        t.rotation.y = p.ori.y
        t.rotation.z = p.ori.z
        t.rotation.w = p.ori.w
        self.publish_tf(t, source_frame, goal_pose_name)

    def publish_tf(self, transform, source_frame_id, child_frame_id):
        """
        Convenience function to publish a transform.
        Args:
            transform: geometry_msgs Transform type, from source_frame to child_frame
            source_frame_id: name of source frame
            child_frame_id: name of child frame
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source_frame_id
        t.child_frame_id = child_frame_id
        t.transform = transform
        self.tf_broadcaster.sendTransform(t)

        # Wait until transform is available
        return self._tf_buffer.can_transform(
            source_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
        )

    def get_relative_effector_pose(
        self, goal_tf, relative=geometry.pose(), override_yaw=True, publish_tf=None, approach_axis=None
    ):
        """
        Get a hsrb_interface.geometry Pose tuple representing a relative pose from the
        goal tf, all in base link frame.
        Override the yaw component of the target to point away from the robot, so the hand
        approaches from the correct direction.
        Args:
            goal_tf: goal tf
            relative: relative hsrb_interface.geometry pose to goal tf to get hand pose
        Grasp pose targets use the HSRB hand-palm link coordinate frame. When the hand is
        in go/neutral position:
            - X (red) points UP from hand-palm link.
            - Y (green) points to the RIGHT of the hand, from the hand camera viewpoint.
            - Z (blue) points OUTWARDS from the hand, in the same direction the
              hand camera looks.
        """

        (trans, lookup_time) = self.lookup_transform(self.BASE_FRAME, goal_tf)

        if trans is None:
            return None

        if override_yaw:
            # Replace transform rotation component with the adjusted version
            # This ensures the hand approaches from the direction of the robot
            yaw = math.atan2(trans.translation.y, trans.translation.x)

            # roll pitch yaw
            # hard coded values are to ensure z direction points away from robot, with x
            # direction pointing upwards
            base_matrix = tf.transformations.euler_matrix(1.57, -1.57, 1.57 + yaw)

            rotation_matrix = tf.transformations.identity_matrix()
            if approach_axis is not None:
                z_axis = base_matrix[:3, 2]

                # Calculate the rotation matrix to let robot aproach from the specified direction
                rotation_axis = np.cross(z_axis, approach_axis)
                rotation_angle = np.arccos(np.dot(z_axis, approach_axis) / (np.linalg.norm(z_axis) * np.linalg.norm(approach_axis)))
                rotation_matrix = tf.transformations.rotation_matrix(rotation_angle, rotation_axis)

            # Apply Rotation
            q = tf.transformations.quaternion_from_matrix(np.dot(rotation_matrix, base_matrix))

            trans.rotation.x = q[0]
            trans.rotation.y = q[1]
            trans.rotation.z = q[2]
            trans.rotation.w = q[3]

        frame_to_ref = geometry.transform_to_tuples(trans)
        frame_to_hand = geometry.multiply_tuples(frame_to_ref, relative)

        if publish_tf is not None:
            relative_transform = geometry.tuples_to_transform(frame_to_hand)
            self.publish_tf(relative_transform, self.BASE_FRAME, publish_tf)

        return frame_to_hand

    def lookup_transform(self, source, dest, timeout=rospy.Duration(0)):
        """
        Lookup a transfrom and its timestamp.
        Args:
            source: name of source frame
            dest: name of dest frame
        Returns: (transform, time) where transform is a geometry_msgs Transform and time
                  is a rostime.Time
        """
        try:
            trans_stamped = self._tf_buffer.lookup_transform(source, dest, rospy.Time(), timeout=timeout)
            return trans_stamped.transform, trans_stamped.header.stamp

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None, None

    def calc_transform_distance(self, t, only_2d=False):
        """
        Calculate the L2 distance represented by a transform.
        Args:
            t: transform
        """
        if only_2d:
            return math.sqrt(
                math.pow(t.translation.x, 2) + math.pow(t.translation.y, 2)
            )
        else:
            return math.sqrt(
                math.pow(t.translation.x, 2)
                + math.pow(t.translation.y, 2)
                + math.pow(t.translation.z, 2)
            )

    def tts_say(self, string_to_say, duration=None):
        """
        Say something via text-to-speech, if text-to-speech narration is enabled.
        Args:
            string_to_say: string to be said out loud by the robot.
        """
        if self.tts_narrate:
            self.tts.say(string_to_say)
            if duration is not None:
                rospy.sleep(duration)

    # Common kinematic functionality

    def finish_position(self, collision_world):
        """
        Try to revert the robot to go position, free of any obstacles.
        """

        try:
            rospy.loginfo(
                "%s: Trying to move back and get into go position." % self._action_name
            )
            with collision_world:
                self.omni_base.go_rel(-0.3, 0, 0)
                self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        # Unsuccessful - try without collision avoidance
        try:
            rospy.loginfo(
                "%s: Retrying without collision detection." % self._action_name
            )
            self.omni_base.go_rel(-0.3, 0, 0)
            self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        # Unsuccessful - try going sideways
        try:
            rospy.loginfo("%s: Trying to move to the side instead." % self._action_name)
            self.omni_base.go_rel(0, 0.3, 0)
            self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        return False

    def move_arm_down(self, arm_lift_joint_height, back_away=True):
        """
        Move backwards and flip arm down.
        """
        if back_away:
            self.omni_base.go_pose(
                geometry.pose(x=-self.ARM_SWING_DIST),
                10.0,
                ref_frame_id=self.BASE_FRAME,
            )

        if arm_lift_joint_height < self.MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION:
            rospy.logerr(
                "%s: Arm lift joint height too low to move arm down" % self._action_name
            )
            return

        self.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": arm_lift_joint_height,
                "arm_flex_joint": self.MIN_ANGLE_ARM_FLEX_HEIGHT,
            }
        )

        if back_away:
            self.omni_base.go_pose(
                geometry.pose(x=self.ARM_SWING_DIST), 10.0, ref_frame_id=self.BASE_FRAME
            )

    def look_at_object(self, object_tf, rotate_to_face=False):
        """
        Get a "best-view" look at the object at the specified tf.
        Args:
            object_transform: tf
            rotate_to_face: whether to rotate whole robot body to face object
        Returns: success, true or false
        """
        (trans, _) = self.lookup_transform(self.BASE_FRAME, object_tf)

        if trans is None:
            return False

        if rotate_to_face:
            yaw = math.atan2(trans.translation.y, trans.translation.x)
            self.omni_base.go_rel(0.0, 0.0, yaw, 10.0)

        object_height = trans.translation.z
        self.move_camera_to_height(object_height + self.LOOK_ANGLE_OFFSET)

        self.whole_body.gaze_point(
            point=geometry.Vector3(0, 0, 0), ref_frame_id=object_tf
        )

        return True

    def move_camera_to_height(self, h):
        """
        Move the RGBD camera to a specified height. Assumes start state is go position.
        Args:
            h: height in metres
        """
        if h <= self.RGBD_MIN_HEIGHT:
            return

        arm_joint_height = (h - self.RGBD_MIN_HEIGHT) * self.H_ARM_RGBD_SCALE_FACTOR

        if arm_joint_height > self.MAX_HEIGHT_ARM_LIFT_JOINT:
            rospy.logwarn(
                "%s: Can't get RGBD camera high enough to look down at goal. Target height %f m"
                % (self._action_name, h)
            )
            arm_joint_height = self.MAX_HEIGHT_ARM_LIFT_JOINT

        if arm_joint_height < self.MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION:
            self.whole_body.move_to_joint_positions(
                {"arm_lift_joint": arm_joint_height}
            )
        else:
            self.move_arm_down(arm_lift_joint_height=arm_joint_height)

    # Common collision world functionality

    def get_goal_cropped_collision_map(self, goal_tf, crop_dist_3d=0.1):
        """
        Get a 2m*2m*2m collision map centred on goal_tf, with the area around goal_tf
        cropped out to a distance of crop_dist_3d.
        """
        rospy.loginfo("%s: Getting Collision Map." % self._action_name)

        (trans, _) = self.lookup_transform(self.MAP_FRAME, goal_tf)
        goal_x = trans.translation.x
        goal_y = trans.translation.y
        goal_z = trans.translation.z

        external_bounding_box = BoundingBox(
            min=Point(goal_x - 1.0, goal_y - 1.0, goal_z - 1.0),
            max=Point(goal_x + 1.0, goal_y + 1.0, goal_z + 1.0),
        )

        crop_bounding_boxes = []
        # NOTE this is a hard-coded axis-aligned goal bounding box
        if crop_dist_3d > 0:
            bound_x = bound_y = bound_z = crop_dist_3d
            object_bounding_box = BoundingBox(
                min=Point(goal_x - bound_x, goal_y - bound_y, goal_z - bound_z),
                max=Point(goal_x + bound_x, goal_y + bound_y, goal_z + bound_z),
            )
            crop_bounding_boxes.append(object_bounding_box)

        collision_world = self.collision_mapper.build_collision_world(
            external_bounding_box, crop_bounding_boxes=crop_bounding_boxes
        )

        rospy.loginfo("%s: Collision Map generated." % self._action_name)

        return collision_world


class ROSServiceContextManager:
    """
    Context manager wrapper to ensure services are called before/after blocks of code.
    """

    def __init__(self, enter_services, exit_services):
        """
        args:
            - enter_services: {service_proxy: request_msg, ...}
            - exit_services: {service_proxy: request_msg, ...}
        """
        self.enter_services = enter_services
        self.exit_services = exit_services

    def __enter__(self):
        # Call enter services
        for (service, message) in self.enter_services.items():
            service.call(message)

    def __exit__(self, type, value, traceback):
        # Call exit services
        for (service, message) in self.exit_services.items():
            service.call(message)
