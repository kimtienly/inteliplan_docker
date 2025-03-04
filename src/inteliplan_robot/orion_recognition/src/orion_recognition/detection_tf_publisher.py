#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs.msg
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import tf2_ros
from orion_actions.msg import DetectionArray


class DetectionTFPublisher(object):
    def __init__(self):
        self.bridge = CvBridge()
        detection_sub = message_filters.Subscriber(
            "/vision/bbox_detections", DetectionArray)
        depth_sub = message_filters.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image)
        self._objects = rospy.get_param('~objects', [])
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [detection_sub, depth_sub], 30, 0.5)
        self._ts.registerCallback(self.callback)
        self._br = tf2_ros.TransformBroadcaster()

    def callback(self, detections, depth_data):
        objects = {key.label.name: [] for key in detections.detections} #self._objects}
        trans = []
        for detection in detections.detections:
                object_point = [detection.translation_x, detection.translation_y, detection.translation_z]
                objects[detection.label.name].append(object_point)
        print(objects)
        
        for obj in objects:
            for i, pos in enumerate(objects[obj]):
                t = geometry_msgs.msg.TransformStamped()
                t.header = depth_data.header
                t.child_frame_id = obj + '_' + str(i)
                t.transform.translation.x = pos[0]
                t.transform.translation.y = pos[1]
                t.transform.translation.z = pos[2]
                # compute the tf frame
                # rotate -90 degrees along z-axis
                t.transform.rotation.z = np.sin(-np.pi / 4)
                t.transform.rotation.w = np.cos(-np.pi / 4)
                trans.append(t)
                #print(trans)
        self._br.sendTransform(trans)


if __name__ == '__main__':
    rospy.init_node('detection_tf_publisher')
    DetectionTFPublisher()
    rospy.spin()