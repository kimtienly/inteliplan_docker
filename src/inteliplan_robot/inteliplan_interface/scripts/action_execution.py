'''
This is the interface to control the robot, which includes:
- communication with orion_manipulation for 'pick' and 'place' actions.
- directly control simple actions: 'turn', 'search', 'go'.
'''

import actionlib
import hsrb_interface
import orion_actions.msg as msg
import math
import rospy
from geometry_msgs.msg import Transform,TransformStamped
import tf2_ros
class HSR_control:
    """
    This class is an interface to HSR, including communicating with Orion manipulation stack and internal command
    """

    def __init__(self):
        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get("whole_body")
        self.omni_base = self.robot.try_get("omni_base")
        self.whole_body.move_to_neutral()
        self.gripper = self.robot.try_get("gripper")
        self.whole_body.move_to_joint_positions({'head_tilt_joint':-math.pi/7})
        self.origin = self.omni_base.pose
        # TF defaults to buffering 10 seconds of transforms. Choose 60 second buffer.
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration.from_sec(60.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def pick_up_object_client(self, object_pose, approach_axis=None, extend_distance=0, is_bin_bag=False, goal_tf = "pick_up_goal"):

        self.publish_goal_pose_tf(object_pose, goal_pose_name=goal_tf)

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

        rospy.sleep(1)
        # Return the result of executing the action
        return client.get_result()

    def put_object_on_surface_client(self, surface_pose, shelf_tf_ref_="", goal_tf = "place_goal"):
        
        self.publish_goal_pose_tf(surface_pose, goal_pose_name=goal_tf)

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
    

    def turn(self, direction):
        if direction == 'left':
            angle = math.pi/2
        elif direction == 'right':
            angle = -math.pi/2
        elif direction == 'around':
            angle = -math.pi
        try:
            self.omni_base.go_rel(0, 0, angle, 50)
            return True
        except:
            return False

    def search(self, obj, vision_func):
        
        delta_rot = math.pi/8
        delta_head = math.pi/17

        # self.omni_base.go_rel(0.8,0,0,50) #testing only, must be deleted
        # for torso in [0.5, 0]:
        # for torso in [0.2]:
            # self.whole_body.move_to_joint_positions({'arm_lift_joint':torso})
        for rot in [-delta_rot,delta_rot,delta_rot]:
        # for rot in [0]:
            self.omni_base.go_rel(0, 0, rot, 10)
            for head in [-delta_head,delta_head]:
                self.whole_body.move_to_joint_positions({'head_tilt_joint':-math.pi/7+head})
                # print('Vision returns ',vision_func(obj))
                rospy.sleep(2)
                if vision_func(obj):
                    print('<vision>: found ')
                    import time
                    time.sleep(2)
                    # rospy.sleep(2)
                    print('Checking feasibility..')
                    print('<feasibility>: 1')
                    return True
                    
        return False

    def move(self, loc, absolute = True):
        try:
            if absolute:
                loc = self.origin
                self.omni_base.go_abs(loc[0],loc[1],loc[2],100)
            else: #relative pose
                self.omni_base.go_rel(loc[0],loc[1],loc[2],100)
            rospy.sleep(1)
            return True
        except:
            return False
        
    def drawer(self, pull_dis,handle_sub = "yolo_pointcloud/detection_poses"):#
        import tf
        import numpy as np
        from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, TransformStamped
        from std_msgs.msg import Header
        from manipulation.collision_mapping import CollisionMapper
        from manipulation.msg import BoundingBox
        from pyquaternion import Quaternion
        import hsrb_interface.geometry as geometry
        

        LOOK_DISTANCE = 0.1
        SAFE_DISTANCE = 0.15
        FINGER_LENGTH = 0.088
        GRASP_DISTANCE = SAFE_DISTANCE-FINGER_LENGTH

        def pose_cb(msg):
            # print('callback')
            global live_poses,live_trans,live_rot,live_time

            live_time = msg.header.stamp.to_sec()

            (trans,rot) = listener.lookupTransform('map', 'head_rgbd_sensor_rgb_frame', msg.header.stamp)
            live_trans = np.array(trans)
            live_rot = Quaternion(rot[3],rot[0],rot[1],rot[2])

            live_poses = [(
                live_trans+live_rot.rotate(np.array([pose.position.x,pose.position.y,pose.position.z])),
                live_rot*Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z),
                np.array([pose.position.x,pose.position.y,pose.position.z]),
                Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z)
                ) for pose in msg.poses]
            

        def get_closest_pose(target):
            global live_poses, live_time
            MAX_TRIES = 20

            time = rospy.Time.now().to_sec()
            i = 0
            while live_poses is None and i < MAX_TRIES:
                rospy.sleep(0.1)
                i += 1
            while live_time<time:
                rospy.sleep(0.1)
            

            ## find closest handle to target
            dist = 100000
            pose = None
            while pose == None and i < MAX_TRIES:
                for p in live_poses:
                    d = np.linalg.norm(p[0]-target)
                    if d < dist:
                        dist = d
                        pose = p
                if pose == None:
                    rospy.sleep(0.1)
                elif pose[1].rotate(np.array([0.0,0.0,1.0])) @ np.array([0.0,0.0,1.0]) >= 0.2:
                    pose == None
                    rospy.sleep(0.1)
                i += 1
            # print(pose)
            if pose == None:
                return (None,None,None,None)
            return pose
            

        def move_base(move):
            if move:
                self.whole_body.linear_weight = 30.0
                self.whole_body.angular_weight = 50.0
            else:
                self.whole_body.linear_weight = 100.0
                self.whole_body.angular_weight = 100.0

        def look_at_hand(look):
            self.whole_body.looking_hand_constraint = look


        def open(target,pull_distance):
            global live_trans, live_rot

            
            ## move to initial pose
            move_base(True)
            look_at_hand(True)
            pos, ori, _, _ = get_closest_pose(target)

            collision_world = get_goal_cropped_collision_map(Point(pos[0],pos[1],pos[2]))
            print('Collision world has been cropped')
            with collision_world:
                pos += ori.rotate(np.array([-LOOK_DISTANCE,0.0,-SAFE_DISTANCE]))
                print("publish initial pose")
                pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                #rospy.sleep(1)
                # self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(pos[0],pos[1],pos[2]),geometry.Quaternion(ori.x,ori.y,ori.z,ori.w))], ref_frame_id='map')
                #rospy.sleep(2)

                ## move to secondary pose
                move_base(False)
                abs_pos, abs_ori, rel_pos, rel_ori = get_closest_pose(target)
                print("publish second pose")
                if rel_pos is not None:
                    pos = abs_pos
                    ori = abs_ori
                    pos += ori.rotate(np.array([0.0,0.0,-SAFE_DISTANCE]))
                    print("Moving rel")
                    rel_pos += rel_ori.rotate(np.array([0.0,0.0,-SAFE_DISTANCE]))
                    pub.publish(Header(frame_id='head_rgbd_sensor_rgb_frame'),Pose(Point(rel_pos[0],rel_pos[1],rel_pos[2]),rel_ori))
                    self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(rel_pos[0],rel_pos[1],rel_pos[2]),geometry.Quaternion(rel_ori.x,rel_ori.y,rel_ori.z,rel_ori.w))], ref_frame_id='head_rgbd_sensor_rgb_frame')
                else:
                    print("Moving abs")
                    pos += ori.rotate(np.array([LOOK_DISTANCE,0.0,0.0]))
                    pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                    self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(pos[0],pos[1],pos[2]),geometry.Quaternion(ori.x,ori.y,ori.z,ori.w))], ref_frame_id='map')
                    

                ## show grasp pose relitive to map
                pos += ori.rotate(np.array([0.0,0.0,GRASP_DISTANCE]))
                print("publish predicted grasp pose")
                pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                #rospy.sleep(1)

                ## show grasp pose relitive to hand
                (trans,rot) = listener.lookupTransform('hand_palm_link', 'map',rospy.Time())
                move = (Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,GRASP_DISTANCE]))
                print("publish target grasp pose")
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(move[0],move[1],move[2]),Quaternion(1,0,0,0)))
                #rospy.sleep(1)

                ## move hand to handle and grab
                self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,1])), GRASP_DISTANCE, ref_frame_id='hand_palm_link')  # 0.058
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(0,0,FINGER_LENGTH),Quaternion(1,0,0,0)))
                # gripper.command(-1)
                self.gripper.apply_force(1)
                # gripper.set_distance(0.0)
                # gripper.apply_force(0.8)

                ## pull handle
                move_base(True)
                (trans,rot) = listener.lookupTransform('hand_palm_link', 'map',rospy.Time())
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(0,0,0),Quaternion(rot[3],rot[0],rot[1],rot[2])*ori))
                self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,-1])), pull_distance, ref_frame_id='hand_palm_link')

                ## let go and return to safe pose
                self.gripper.command(1.2)
                # self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,-1])), SAFE_DISTANCE, ref_frame_id='hand_palm_link')        

        def get_goal_cropped_collision_map(goal, crop_dist_3d=0.1):
            """
            Get a 2m*2m*2m collision map centred on goal_tf, with the area around goal_tf
            cropped out to a distance of crop_dist_3d.
            """

            goal_x = goal.x
            goal_y = goal.y
            goal_z = goal.z

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

            (trans,rot) = listener.lookupTransform('map','head_rgbd_sensor_rgb_frame' ,rospy.Time())
            bound_x = bound_y = bound_z = 0.01
            head_bounding_box = BoundingBox(
                min=Point(trans[0] - bound_x, trans[1] - bound_y, trans[2] - bound_z),
                max=Point(trans[0] + bound_x, trans[1] + bound_y, trans[2] + bound_z),
            )
            crop_bounding_boxes.append(head_bounding_box)
            bound_x = bound_y = bound_z = 0.01
            floor_bounding_box = BoundingBox(
                min=Point(goal_x - 1, goal_y - 1, -bound_z),
                max=Point(goal_x + 1, goal_y + 1, bound_z),
            )
            crop_bounding_boxes.append(floor_bounding_box)

            collision_world = collision_mapper.build_collision_world(
                external_bounding_box, crop_bounding_boxes=crop_bounding_boxes
            )

            return collision_world
        global live_poses, live_time
        live_poses = None
        live_time = 0
        i=1
        listener = tf.TransformListener()
        transformer = tf.Transformer()
        pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

        pose_sub = rospy.Subscriber("yolo_pointcloud/detection_poses", PoseArray, pose_cb, queue_size=1)
        targets=[np.array([1.05,0,0.25]),np.array([1.05,0,0.5]),np.array([1.05,0,0.5]),np.array([1.05,0,0.25]),np.array([1.05,0,0.7]),np.array([1.05,0,0.7])]
        collision_mapper = CollisionMapper(self.robot)
        self.whole_body.move_to_neutral()
        move_base(True)
        look_at_hand(False)
        self.gripper.command(1.2)
        self.omni_base.go_abs(0, 0, 0, timeout=40.0)
        self.whole_body.gaze_point(targets[i], ref_frame_id='map')
        print('planning action..')
        open(targets[i],pull_dis)
        self.whole_body.move_to_neutral()

    def publish_goal_pose_tf(
        self, p, goal_pose_name
    ):
        t = Transform()
        t.translation.x = p[0]
        t.translation.y = p[1]
        t.translation.z = p[2]
        t.rotation.x = 0
        t.rotation.y = 0
        t.rotation.z = 0
        t.rotation.w = 1
        self.publish_tf(t, 'map', goal_pose_name)

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
    

if __name__=='__main__':
    robot = HSR_control()
    # robot.put_object_on_surface_client([1.7, 0.9, 0.55])
    # robot.open_drawer()
    # robot.turn('left')
    # robot.whole_body.move_to_joint_positions({
    #         'head_tilt_joint':-math.pi/9,
    #     })
