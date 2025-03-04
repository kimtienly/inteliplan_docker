Branch `noetic` is based off `clarissa`, but is changed over to python3 instead of python2.

# orion_recognition

```
$ sim_mode
<local>~$ source catkin_ws/devel/setup.bash
```
or 
```
$ hsrb_mode
<hsrb>~$ source catkin_ws/devel/setup.bash
```
## Object detection

### BboxPublisher Node
The node file is located at `src/orion_recognition/bbox_publisher.py`. 
The node can be spun up using the set-up file here: `scripts/bbox_publisher_node.py`

#### Subscribers and Publishers
BboxPublisher subscribes to the image_topic and the depth_topic. The two subscriptions are synchronized to make sure the image and the depth come from similar times. 

It publishes two items, one being a DetectionArray of the detections made in a frame(`/vision/bbox_detections/`), and the frame image with bounding boxes overlayed(`/vision/bbox_image`).

#### Detection
A single frame from the image_topic is sent to an `ObjectDetector`. This returns a dictionary of the boxes, labels and scores of the detections made. The box is defined by a tuple of the x-y values of the top left corner and the bottom right corner.

We then use the middle point of the box to determine the distance of the object, and the depth of the object is determined by the average of the width and the high of the box.The colour of the object is determined as the average of the the pixels in the box. 

This is used to create a Detection instance, as defined in `orion_actions`. The detections are then filtered for their score, and then grouped into detections with the same labels. The detections with the same labels undergo non-maximum supression.

The resulting `clean_detections` are then used to create our frame with bounding boxes overlayed, and the DetectionArray which will be published. 

#### Changeable Parameters

1. `min_acceptable_score`: The minimum score a detection can have to be considered to be valid. Between 0 and 1.
2. `iou_threshold`: The intersection-over-union threshold, defines how much two boxes need to overlap before being considered to be the same image. The higher it is, a larger proportion needs to overlap. Between 0 and 1.
3. `z_size`: The relationship between the width and height of the object and it's depth is defined here. Default assumes that the object is a cube, change if dimensions are known.


### Object Detector

#### Package Info
Located at `src/orion_recognition/object_detector.py`.
Takes in the image as a tensor, and passes it though a pre-trained object detector. 
We have two object detectors - Faster R-CNN and YoLo v5 (recommended). These produce the bounding boxes, the labels and scores detected in the frame. The labels are numerical values that correspond to a string in the file `coco_labels2017.txt`. 
In Faster R-CNN, the label numbers are offsetted by 1 (label #1 corresponds to a human detection).
Non-human detections are parsed through to an objected classifer, which has been trained by Shu. The object classifer takes in a section of the frame, and returns the most likely object in the image. 

#### Changeable Parameters
1. `use_classifier` : A boolean determining whether the classifer will be used or not. 
2. `Buffer`: The number of pixels added to the edge of the bounding box of an detected object before it is parsed to the object classifer. 

