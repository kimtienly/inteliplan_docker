#!/usr/bin/env python3

from pose_detector_ros import PoseDetector
from orion_actions.msg import PoseDetection, PoseDetectionArray
import pdb
import cv2
import numpy as np
import os
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import ImageDraw, ImageFont
from PIL import Image as Im_PIL
from colornames import ColorNames
   
class PosePublisher(object):
    def __init__(self, image_topic):
        self.detector = PoseDetector()
        #rospy.init_node('bbox', anonymous=True)
        self.subscriber = rospy.Subscriber(image_topic, Image, self.callback, queue_size=100)
        self.bbox_pub = rospy.Publisher('/vision/pose_image', Image, queue_size=100)
        self.detections_pub = rospy.Publisher('/vision/pose_detections', PoseDetectionArray, queue_size=100)
        self.bridge = CvBridge()
        #self.font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 50, encoding="unic")


    def callback(self, ros_image):
        image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        np_image = np.asarray(image)
        
        try:
            img,outputs = self.detector.detect(np_image)
            #print(outputs)
            if isinstance(outputs, int):
                print ('No Detection')
                img=image
            else:
                detections = []
                for det in outputs.keys():
                    people_id = int(det)
                    Nose = outputs[det][0]['Nose']
                    LEye = outputs[det][0]['LEye']
                    REye = outputs[det][0]['REye']
                    LEar = outputs[det][0]['LEar']
                    REar = outputs[det][0]['REar']
                    LShoulder = outputs[det][0]['LShoulder']
                    RShoulder = outputs[det][0]['RShoulder']
                    LElbow = outputs[det][0]['LElbow']
                    RElbow = outputs[det][0]['RElbow']
                    LWrist = outputs[det][0]['LWrist']
                    RWrist = outputs[det][0]['RWrist']
                    LHip = outputs[det][0]['LHip']
                    RHip = outputs[det][0]['RHip']
                    LKnee = outputs[det][0]['LKnee']
                    RKnee = outputs[det][0]['RKnee']
                    LAnkle = outputs[det][0]['LAnkle']
                    RAnkle = outputs[det][0]['RAnkle']
                    Neck = outputs[det][0]['Neck']
                    upLeft = outputs[det][1]
                    bottomRight= outputs[det][2]
                    if LShoulder[0]!=-1 and LShoulder[1]!=-1 and RShoulder[0]!=-1 and RShoulder[1]!=-1 and Neck[0]!= -1 and Neck[1] != -1 and LHip[0]!=-1 and LHip[1]!=-1:
                        left_point = min(LShoulder[0],RShoulder[0])
                        right_point = max(LShoulder[0],RShoulder[0])
                        top_point = min(Neck[1],LHip[1])
                        bottom_point = max(Neck[1],LHip[1])
                        crop = np_image[int(top_point):int(bottom_point),int(left_point):int(right_point)]
                        #print('box:',left_point,right_point,top_point,bottom_point)
                        RGB = np.mean(crop,axis=(0,1))
                        RGB = (RGB[2], RGB[1], RGB[0])
                        cv2.rectangle(img, (left_point,top_point), (right_point,bottom_point), (0, 0, 255), 5)
                    else:
                        RGB = (-1,-1,-1)
                    color = ColorNames.findNearestOrionColorName(RGB)
                    if min(LWrist[1],RWrist[1]) < min(LEye[1],REye[1]):
                        waving=True
                    else:
                        waving=False
                    #print(LHip[1],LKnee[1])
                    #print(RHip[1],RKnee[1])
                    if max(np.linalg.norm(LHip[1]-LKnee[1]),np.linalg.norm(RHip[1]-RKnee[1]))<90:
                        
                        sitting=True
                    else:
                        sitting=False
                    detection = PoseDetection(people_id, Nose[0], Nose[1], LEye[0], LEye[1], REye[0], REye[1], LEar[0], LEar[1], REar[0], REar[1], LShoulder[0], LShoulder[1],RShoulder[0],RShoulder[1],LElbow[0],LElbow[1],RElbow[0],RElbow[1],LWrist[0],LWrist[1],RWrist[0], RWrist[1],LHip[0],LHip[1],RHip[0],RHip[1], LKnee[0], LKnee[1],RKnee[0],RKnee[1],LAnkle[0],LAnkle[1],RAnkle[0],RAnkle[1],Neck[0],Neck[1],upLeft[0],upLeft[1],bottomRight[0],bottomRight[1],RGB[0],RGB[1],RGB[2],color,waving,sitting)
                    detections.append(detection)
    
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                self.detections_pub.publish(PoseDetectionArray(h, detections))
            self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
                print(e)

 
if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    img_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    pub = BboxPublisher(img_topic)
    rospy.spin() 
