#!/usr/bin/env python3
# Extend from kinova_robot_interface for Anymal control
import rospy
import numpy as np
from rospy.core import deprecated
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math
from urdf_parser_py.urdf import URDF
from collections import OrderedDict
from copy import deepcopy
from angles import shortest_angular_distance, shortest_angular_distance_with_large_limits
from anymal_msgs.msg import AnymalState
from geometry_msgs.msg import PoseStamped
from . import configuration
from . import logger
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import signal
# from multi_gait_executor_msgs.msg import *
from base_controller import base_controller
def signal_handler(sig, frame):
    sys.exit(0)


class RobotInterface:
    def __init__(self,
                 kinova_command_topic: str = '/j1n6s300/position_controller/command',
                 kinova_joint_state_topic: str = '/j1n6s300/joint_states'):
        
        anymal_state_topic = '/state_estimator/anymal_state'

        self.q_measured = np.zeros(6, dtype=np.float64)
        
        self.publisher = rospy.Publisher(kinova_command_topic, Float64MultiArray, queue_size=10)
        self.js_sub = rospy.Subscriber(kinova_joint_state_topic, JointState, self.js_kinova_callback, queue_size=1)

        self.js_sub = rospy.Subscriber(anymal_state_topic, AnymalState, self.js_anymal_callback, queue_size=1)

        # Base position controller
        self.base_controller_ = base_controller.BaseController()

        self.q_nominal = np.array([0.0, 2.9, 1.3, -2.07, 1.4, 0.0])

        self.lastJointValsSent = np.zeros(6, dtype=np.float64)
        self.position_command_msg = Float64MultiArray()
        self.position_command_msg.data = [0.] * 6
        self.first_command = True

        # Get the URDF to identify which joints are continuous
        robot = URDF.from_parameter_server(key=configuration.kinovaDescriptionKey)
        self.joints = []
        self.joint_map = OrderedDict()
        self.nq = 0
        self.lowest_joint_velocity_limit = 1000.
        for joint in robot.joints:
            # print(joint.name)
            if joint.joint_type != 'fixed' and 'finger' not in joint.name:
            # if joint.joint_type != 'fixed' and 'finger' not in joint.name and 'j1' in joint.name:
                # print(joint.name)
                j = {
                    'joint_type': joint.joint_type,
                    'q_idx_start': deepcopy(self.nq),
                    'q_idx_length': 2 if joint.joint_type == 'continuous' else 1,
                    'velocity_limit': joint.limit.velocity
                }
                if joint.type != 'continuous':
                    j['left_limit'] = joint.limit.lower
                    j['right_limit'] = joint.limit.upper
                self.lowest_joint_velocity_limit = min(self.lowest_joint_velocity_limit, joint.limit.velocity)
                self.joints.append(j)
                self.joint_map[joint.name] = j
                if joint.type == 'revolute':
                    self.nq += 1
                elif joint.type == 'continuous':
                    self.nq += 2
        # print("lowest_joint_velocity_limit", self.lowest_joint_velocity_limit)
        print(self.joints)
        self.logging = False
        self.isWalking = False

    def addLogger(self, robotLogger):
        self.logging = True
        self.robotLogger = robotLogger

    def js_kinova_callback(self, data):
        # self.q_measured[-6:] = data.position[:6]

        # We need to wrap joint angles here to avoid weird planning failures
        # This is a simulation-only issue due to a Gazebo initialisation issue
        def wrap(position):
            for i in [0, 3, 4, 5]:
                position[i] = (position[i] + np.pi) % (2 * np.pi) - np.pi
        position = list(data.position)
        wrap(position)

        self.q_measured[18:24] = position[:6]
        self.q_measured[-6:] = 0  # fingers, most likely?



    def js_anymal_callback(self, data):
        euler = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.q_measured[:6] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z]+list(euler)
        self.q_measured[6:18] = data.joints.position[:]

    def distance_between_configurations(self, q1: np.array, q2: np.array) -> np.array:
        """Computes the distance to two configurations considering the continuous joints (first 3 joints)"""
        assert q1.shape[0] == 6
        assert q2.shape[0] == 6
        q1_mfd = np.zeros(self.nq, dtype=np.float64)
        q2_mfd = np.zeros(self.nq, dtype=np.float64)
        for i in range(6):
            joint = self.joints[i]
            # Revolute joints: Absolute
            if joint['joint_type'] == 'revolute':
                q1_mfd[joint['q_idx_start']] = q1[i]
                q2_mfd[joint['q_idx_start']] = q2[i]
            # Continuous joints: SO(2) can be represented as cos(phi),sin(phi)
            elif joint['joint_type'] == 'continuous':
                q1_mfd[joint['q_idx_start'] + 0] = np.cos(q1[i])
                q1_mfd[joint['q_idx_start'] + 1] = np.sin(q1[i])
                q2_mfd[joint['q_idx_start'] + 0] = np.cos(q2[i])
                q2_mfd[joint['q_idx_start'] + 1] = np.sin(q2[i])
        return q1_mfd - q2_mfd

    def distance_to_configuration_target(self):
        """Computes the distance to the configuration target considering the continuous joints (first 3 joints)"""
        return np.linalg.norm(self.distance_between_configurations(self.lastJointValsSent, self.q_measured[18:24]))

    def writeToJointState(self, jointVals, synchronous=True, error_threshold=1):
        """Writes directly to the joints in the kinova arm!"""
        if type(jointVals) == list:
            assert len(jointVals) == 6
        elif type(jointVals) == np.ndarray:
            assert jointVals.shape[0] == 6
        else:
            print("writeToJointState type = ", type(jointVals))

        self.lastJointValsSent[:] = jointVals
        self.pushJointState()

        # Yield until reached
        if synchronous:
            r = rospy.Rate(100)
            s = time.perf_counter()
            while self.distance_to_configuration_target() > error_threshold:
                self.pushJointState()
                if time.perf_counter() - s > 4.0:
                    rospy.logwarn_throttle(1, "Still waiting to reach desired configuration, error = {0:.3}".format(
                        self.distance_to_configuration_target()))
                    # print("q_target", self.lastJointValsSent)
                    # print("q_measured", self.q_measured)
                    # print("q_error", self.distance_between_configurations(self.lastJointValsSent, self.q_measured))
                r.sleep()

    def pushJointState(self):
        self.position_command_msg.data[0:6] = self.lastJointValsSent[0:6]
        self.publisher.publish(self.position_command_msg)

        if self.first_command:  # Push twice because of ROS bug
            self.publisher.publish(self.position_command_msg)
            self.first_command = False

    def moveKinovaToConfigurationWithVelocityInterpolation(
            self,
            q_target,
            max_vel=None,
            funcPropTo=lambda x: 1 - math.cos(x)):

        q_start = self.q_measured[18:24].copy()

        max_vel = self.lowest_joint_velocity_limit if max_vel is None else max_vel

        # We need to compute an difference vector that represents the shortest distance considering joint limits and continuous joints
        q_diff_shortest_angular_distance = np.zeros(6, dtype=np.float64)
        for i in range(6):
            if self.joints[i]['joint_type'] == 'continuous':
                q_diff_shortest_angular_distance[i] = shortest_angular_distance(q_start[i], q_target[i])
            else:
                q_diff_shortest_angular_distance[i] = \
                shortest_angular_distance_with_large_limits(q_start[i], q_target[i],
                                                            self.joints[i]['left_limit'],
                                                            self.joints[i]['right_limit'])[1]

        # We want the max speed of the wrist joints to be twice that of the elbow or shoulder
        # because per radian of wrist motion, you get a much smaller motion of the hand than
        # per radian of shoulder or elbow.
        wrist_multiplier = 2
        max_change = 0
        for i in range(3):
            if (np.abs(q_diff_shortest_angular_distance)[i] > max_change):
                max_change = np.abs(q_diff_shortest_angular_distance)[i]
        for i in range(3, 6):
            if (np.abs(q_diff_shortest_angular_distance)[i] / wrist_multiplier > max_change):
                max_change = np.abs(q_diff_shortest_angular_distance)[
                                 i] / wrist_multiplier  # We want to halve it so that it has half the effect
        # max_change = np.abs(q_diff_shortest_angular_distance).max()
        time_for_motion_considering_max_vel = max_change / max_vel

        if (time_for_motion_considering_max_vel < 2):
            time_for_motion_considering_max_vel = 2

        rate = 120
        r = rospy.Rate(rate)
        steps = math.ceil(time_for_motion_considering_max_vel / (1 / rate))
        for i in range(steps):
            t = np.pi / 2.0 * i / steps  # We move from 0->pi/2 in "steps" steps
            q_interpolated = q_start + funcPropTo(t) * q_diff_shortest_angular_distance
            self.writeToJointState(q_interpolated, synchronous=False)
            r.sleep()

    def moveKinovaToConfigurationWithVelocity_sinSquaredInterpolation(self, q_target, max_vel=None):
        self.moveKinovaToConfigurationWithVelocityInterpolation(
            q_target,
            max_vel,
            funcPropTo=lambda x: math.sin(x) * math.sin(x))

    def moveThroughPathSet(self, pathSet,baseMovement,baseHeight=0.48):
        """The idea here is that you have an array of paths, where each path is a set of positions
        for the kinova to move through."""

        rospy.loginfo("Moving through path...")
        # self.moveKinovaToConfigurationWithVelocity_sinSquaredInterpolation(pathSet[0][len(pathSetath[0]) - 1])
        # rate = rospy.Rate(70)
        # # rate = rospy.Rate(100)
        # rate.sleep()
        # self.robotLogger.updateStep()
        # pathSet = pathSet[4:-2]

        for index, path in enumerate(pathSet):

            rospy.loginfo("\tPath " + str(index) + " in " + str(len(pathSet)) + " paths.")
            rate = rospy.Rate(100)
            # print(path)
            if baseMovement is None:
                self.walkToBasePose([path[0],path[1],baseHeight,0,0,path[2]], yawoffset=0)
                # if not configuration.online_planning:
                # else:
                #     self.walkToBasePose(path[:6], yawoffset=0)
                #     print('input:',path[:6])
                #     print('actual:',self.q_measured[:6])
                #     basereach = True
            else:
                if baseMovement[index]==1:
                    temp = [1.57, 2.0, 1.4, 0.0, 0.6, -0.5]
                    self.writeToJointState(temp, synchronous=False)
                    print('moving base at step ',index)
                    # self.walkToBasePose(path[:6],yawoffset=0)
                    path = path.tolist()
                    modified_base = [path[0]]+[path[1]]+path[2:6]
                    modified_base = np.asarray(modified_base)
                    self.walkToBasePose(modified_base,yawoffset=0)
                    print('input:',modified_base)
                    print('actual:',self.q_measured[:6])
        
            rate.sleep()
            # print('reached base at ',self.q_measured[:6])
            # if basereach:
            try:
                # print('writing to joint state',path[-6:])
                self.writeToJointState(path[-6:], synchronous=True, error_threshold=0.02)
                rate.sleep()
                # print('reached arm joints at ', self.q_measured[18:24])
                #self.robotLogger.updateStep()
            except KeyboardInterrupt:
                break
            # time.sleep(0.5)

    def moveToNominalConfiguration(self):
        rospy.loginfo("Moving to nominal configuration")
        self.moveToConfigurationWithVelocityInterpolation(self.q_nominal)

    def walkToBasePose(self,pose,yawoffset=1.57):        # rospy.init_node('multi_gait_executor_demo')
        self.isWalking = True
        self.base_controller_.executeTasks(pose)
        self.isWalking = False

