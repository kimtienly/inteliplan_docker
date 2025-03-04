#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import torch
import cv2
import numpy as np

from pathlib import Path
from ultralytics import YOLO  # YOLOv8

# tensorrt
from models import TRTModule  # isort:skip

from trackers.multi_tracker_zoo import create_tracker
from yolov8.ultralytics.yolo.utils.plotting import Annotator, colors

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] 
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'trackers' / 'strongsort') not in sys.path:
    # add strong_sort ROOT to PATH
    sys.path.append(str(ROOT / 'trackers' / 'strongsort'))


class ObjectDetector(torch.nn.Module):
    def __init__(self):
        
        # This is a temp solution, should change to a rosparam???
        self.model_path = "/home/ori/orion_yolo_robocup/"#Path(__file__).resolve().parent.parent.parent
        self.model_name = "yolov8l-seg.engine"

        super(ObjectDetector, self).__init__()
        print("Is cuda available? {}".format(torch.cuda.is_available()))
        print("cuda memory allocated: {}".format(
            torch.cuda.memory_allocated()))  # does not cause seg fault
        # print(torch.cuda.memory_allocated(0)) # causes seg fault
        self.device = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu")

        try:
            self.model = YOLO(os.path.join(self.model_path, self.model_name)) # load saved copy of pretrained weights
        except:
            self.model = YOLO('yolov8l.pt') # download weights

        # Send device to model
        Engine = TRTModule(self.model, self.device)
        H, W = Engine.inp_info[0].shape[-2:]
        # set desired output names order
        Engine.set_desired(['outputs', 'proto'])
        # Dictionary object, key is the index, value is the corresponding string object class
        self.label_map = self.model.names


    def detect_img_single(self, img_source, show_result=False):
        """
        Detect obj on a single image
        
        some Supported img_source
        (Ref: https://docs.ultralytics.com/modes/predict/)
        Source name:        |       Type:
        path_to_img                 str, Path
        OpenCV                      np.ndarray
        numpy                       np.ndarray
        torch                       torch.Tensor
        """
        # tensorrt inference here
        result_yolo = self.model(img_source)
        if show_result == True:
            # Read img data

            if isinstance(img_source, str):
                img_temp = cv2.imread(img_source)

            elif isinstance(img_source, np.ndarray):
                img_temp = img_source

            else:
                raise NotImplementedError

            # Visualise the results on the frame
            
            annotated_frame = result_yolo[0].plot()

            # Display the annotated frame
            cv2.imshow("YOLOv8 Inference", annotated_frame)

            # Close window for any key pressed
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()

        return result_yolo



    def detect_webcam(self, webcam_no=2):
        """
        Start a webcam, and detect objects on the video        
        
        """
        cv2.namedWindow("Webcam")
        vc = cv2.VideoCapture(webcam_no)

        while vc.isOpened():  # try to get the first frame
            rval, frame = vc.read()
            if not rval:
                break
            print(frame)
            result_temp = self.detect_img_single(frame)
            
            # Visualize the results on the frame
            annotated_frame = result_temp[0].plot()
            
            cv2.imshow("Webcam", annotated_frame)
            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                break

        vc.release()
        cv2.destroyWindow("Webcam")



    def detect_and_track_webcam(
            self, 
            webcam_no=2,
            tracking_method='strongsort',
            reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt',  # model.pt path
            half=False,  # use FP16 half-precision inference
            hide_labels=False,  # hide labels
            hide_conf=False,  # hide confidences
            hide_class=False,  # hide IDs
            ):
        
        """
        Start a webcam, and detect objects on the video

        """
        tracking_config = ROOT / 'trackers' / tracking_method / 'configs' / (tracking_method + '.yaml')

        tracker = create_tracker(
            tracking_method, tracking_config, reid_weights, self.device, half)
        if hasattr(tracker, 'model'):
            if hasattr(tracker.model, 'warmup'):
                tracker.model.warmup()
        outputs = [None]

        curr_frame, prev_frame = [None], [None]

        cv2.namedWindow("Webcam")
        vc = cv2.VideoCapture(webcam_no)

        while vc.isOpened():  # Try to get the first frame
            rval, frame = vc.read()
            if not rval:
                break
            

            result_temp = self.detect_img_single(frame)

            # # Visualise the results on the frame with YOLOv8 tool
            # annotated_frame = result_temp[0].plot()

            names = result_temp[0].names
            annotator = Annotator(frame, line_width=2, example=str(names))

            curr_frame = frame.copy()

            if hasattr(tracker, 'tracker') and hasattr(tracker.tracker, 'camera_update'):
                # Camera motion compensation
                if prev_frame is not None and curr_frame is not None:
                    tracker.tracker.camera_update(prev_frame, curr_frame)

            if result_temp is not None and len(result_temp):
                detections = result_temp[0].boxes.boxes
                outputs = tracker.update(detections.cpu(), frame)

                # Draw boxes for visualisation
                if len(outputs) > 0:
                    for j, (output) in enumerate(outputs):

                        bbox = output[0:4]
                        id = output[4]
                        cls = output[5]
                        conf = output[6]


                        c = int(cls)  # integer class
                        id = int(id)  # integer id
                        label = None if hide_labels else (f'{id} {names[c]}' if hide_conf else
                                                        (f'{id} {conf:.2f}' if hide_class else f'{id} {names[c]} {conf:.2f}'))
                        color = colors(c, True)
                        annotator.box_label(bbox, label, color=color)

                        # to MOT format
                        bbox_left = output[0]
                        bbox_top = output[1]
                        bbox_w = output[2] - output[0]
                        bbox_h = output[3] - output[1]

                        # Visualise trajectories of the detections
                        if tracking_method == 'strongsort':
                            q = output[7]
                            tracker.trajectory(frame, q, color=color)

                # print('outputs', outputs)

            prev_frame = curr_frame

            # Stream results
            frame = annotator.result()

            # Visualise YOLOv8 detections
            # cv2.imshow("Webcam", annotated_frame)

            # Visualise tracker detections and trajectories
            cv2.imshow("Webcam", frame)

            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                break

        vc.release()
        cv2.destroyWindow("Webcam")


    def decode_result_Boxes(self, result_YOLOv8):
        """
        Decode YOLOv8's Results object into Boxes object
        Return bbox_results and result_Boxes

        bbox_results = {
            'boxes': [],        #  in xyxy format
            'scores': [],       # confidence score
            'labels': []        # string class
        }


        result_Boxes is the Boxes object defined in YOLOv8
        """
        
        
        if len(result_YOLOv8) != 1:
            raise Exception("Error, should only have ONE result")
            

        result_Boxes = result_YOLOv8[0].boxes

        bbox_results_dict = {
            'boxes': [],  # in xyxy format
            'scores': [],       # confidence score
            'labels': []        # string class
            }

        for i in range(len(result_Boxes)):
            bbox_results_dict["boxes"].append(result_Boxes[i].xyxy.numpy())
            bbox_results_dict["scores"].append(result_Boxes[i].conf.numpy())
            bbox_results_dict["labels"].append(
                self.label_map[result_Boxes[i].cls.numpy()[0]])

        return bbox_results_dict, result_Boxes


if __name__ == '__main__':
    detector = ObjectDetector()

    detector.detect_webcam(0)
    #re = detector.detect_img_single("/home/jianeng/Pictures/test/fox.jpg")
    #     "/home/ana/Desktop/Orion/orion_recognition/src/tmp.jpg", show_result=True)
    # bbox_temp = detector.decode_result_Boxes(re)[0]

    # detector.detect_and_track_webcam(0)
    # detector.detect_webcam(0)
