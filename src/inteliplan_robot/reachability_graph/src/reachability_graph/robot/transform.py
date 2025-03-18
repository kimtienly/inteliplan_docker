import tf2_ros
import numpy as np
import rospy
import open3d as o3d
from geometry_msgs.msg import Pose
import pyexotica as exo

from .. import configuration

class KinovaTransformManager:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # trans = self.listener.waitForTransform(ref_frame, init_frame, rospy.Time(0), rospy.Duration(5))
        self.preMult = np.identity(4)
        self.postMult = np.identity(4)


    def getCurrentCameraTransform_QuatTransl(self):

        trans = self.tfBuffer.lookup_transform(
            "camera_depth_optical_frame",
            configuration.globalFrame,
            rospy.Time())

        return trans

    def getCurrentTranslation(self):
        trans = self.getCurrentCameraTransform_QuatTransl()
        return np.asarray([
            trans.transform.translation.x, 
            trans.transform.translation.y, 
            trans.transform.translation.z])

    def getCurrentCameraTransform_Pose(self):
        # Taken from planning_scene_utils.py::create_pose()

        trans = self.getCurrentCameraTransform_QuatTransl()
        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation.w = trans.transform.rotation.w
        pose.orientation.x = trans.transform.rotation.x
        pose.orientation.y = trans.transform.rotation.y
        pose.orientation.z = trans.transform.rotation.z
        return pose

    def getUnitPose(self):
        pose = Pose()
        pose.position.x = pose.position.y = pose.position.z = 0
        pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
        pose.orientation.w = 1
        return pose


    def getCurrentCameraTransform(self):
        trans = self.getCurrentCameraTransform_QuatTransl()
        # print('before ',trans)
        _7vec = np.asarray([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w])

        transformToGlobal = exo.KDLFrame(_7vec).get_frame()

        # There are transformations to do before and after!
        transformToGlobal = np.matmul(self.postMult, np.matmul(transformToGlobal, self.preMult))
        # print('after ',transformToGlobal)
        return transformToGlobal

    def getCurrentEndEffector(self,ref_frame=None):
        if ref_frame is None:
            ref_frame = configuration.globalFrame
        endeff="j1n6s300_end_effector"

        if configuration.robotiq:
            endeff="end_effector"
        # if configuration.SIMULATION:
        #     endeff="j2n6s300_end_effector"
        if configuration.building_reachability_map:
            endeff = "j2n6s300_end_effector"
        trans = self.tfBuffer.lookup_transform(
            ref_frame,
            endeff,
            rospy.Time())

        return np.asarray([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z])

    def getAnymalBasePose(self):
        trans = self.tfBuffer.lookup_transform(
            'map',
            'base',
            rospy.Time())

        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation.w = trans.transform.rotation.w
        pose.orientation.x = trans.transform.rotation.x
        pose.orientation.y = trans.transform.rotation.y
        pose.orientation.z = trans.transform.rotation.z
        return pose

    def getAnymalBasePosition(self):
        trans = self.tfBuffer.lookup_transform(
            'map',
            'base',
            rospy.Time())

        return np.asarray([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z])

    def getBasetoMount(self):
        trans = self.tfBuffer.lookup_transform(
            'base',
            'kinova_mount',
            rospy.Time())

        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation.w = trans.transform.rotation.w
        pose.orientation.x = trans.transform.rotation.x
        pose.orientation.y = trans.transform.rotation.y
        pose.orientation.z = trans.transform.rotation.z
        return pose

    def getTransform(self,a,b):
        trans = self.tfBuffer.lookup_transform(
            a,
            b,
            rospy.Time())

        return np.asarray([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z])


if __name__ == "__main__":
    pass
