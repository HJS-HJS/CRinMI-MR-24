import os
import sys
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from PIL import Image

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir + '/yolov8')

class SegmentInterface():
    
    def __init__(self):
        self.model_path = os.path.join(current_dir, "yolov8/best_1280.pt")
        self.model = YOLO(self.model_path)
        print("Segmentation model ready..")


    def run(self, img):
        results = self.model.predict(img, conf=0.6)
        self.result = results[0]
        self.seg_classes = list(self.result.names.values())
        print("\n", "Available class List", "\n", self.seg_classes)

        # im = Image.fromarray(self.result.plot()[..., ::-1])
        # im.show()

    def img2SegmentMask(self):
        masks_ws_labels = []
        for mask, workspace, label in zip(self.result.masks.data, self.result.boxes.xyxy.tolist(), self.result.boxes.cls):
            # Convert the mask to a binary mask (if necessary)
                x1, y1, x2, y2 = workspace
                binary_mask = (mask > 0.5).cpu().numpy().astype(np.uint8)
                binary_mask = cv2.resize(binary_mask, (1280, 720))
                
                # Append the mask and its corresponding label to the list
                masks_ws_labels.append((binary_mask, [x1, y1, x2, y2], int(label)))

        return masks_ws_labels
        