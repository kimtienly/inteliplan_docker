#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  5 19:57:40 2023

@author: jianeng
"""

from groundingdino.util.inference import load_model, load_image, predict, annotate
import groundingdino.datasets.transforms as T
import cv2
from PIL import Image


transform = T.Compose(
    [
        T.RandomResize([500], max_size=500),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ]
)



path_cfg = "/home/ori/orion_yolo_robocup/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
path_weight = "/home/ori/orion_yolo_robocup/weights/groundingdino_swint_ogc.pth"


model = load_model(path_cfg, path_weight)
#IMAGE_PATH = "weights/dog-3.jpeg"
TEXT_PROMPT = "chair . person . phone ."
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25



cv2.namedWindow("Webcam")
vc = cv2.VideoCapture(0)

while vc.isOpened():  # try to get the first frame
    rval, frame = vc.read()
    if not rval:
        break
    
    im = Image.fromarray(frame)
    image_transformed, _ = transform(im, None)
    
    #image_source, image = load_image(IMAGE_PATH)
    
    boxes, logits, phrases = predict(
        model=model,
        image=image_transformed,
        caption=TEXT_PROMPT,
        box_threshold=BOX_TRESHOLD,
        text_threshold=TEXT_TRESHOLD,
        device="cuda"
    )
    
    annotated_frame = annotate(image_source=frame, boxes=boxes, logits=logits, phrases=phrases)
    
    
    cv2.imshow("Webcam", annotated_frame)
    key = cv2.waitKey(20)
    if key == 27:  # exit on ESC
        break

    vc.release()
    cv2.destroyWindow("Webcam")


