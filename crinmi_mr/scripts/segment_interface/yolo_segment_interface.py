import os
import sys
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from PIL import Image
from scipy.stats import mode

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir + '/yolov8')

class SegmentInterface():
    
    def __init__(self):
        self.model_path = os.path.join(current_dir, "yolov8/best_1280.pt")
        self.model = YOLO(self.model_path)
        print("Segmentation model ready..")


    def run(self, img, vis:bool = True):
        results = self.model.predict(img, conf=0.6)
        self.result = results[0]
        self.seg_classes = list(self.result.names.values())
        print("\n", "Available class List", "\n", self.seg_classes)

        if vis:
            im = Image.fromarray(self.result.plot()[..., ::-1])
            im.show()

    def img2SegmentMask(self):
        masks_ws_labels = []
        for mask, workspace, label in zip(self.result.masks.data, self.result.boxes.xyxy.tolist(), self.result.boxes.cls):
            # Convert the mask to a binary mask (if necessary)
                x1, y1, x2, y2 = workspace
                binary_mask = (mask > 0.5).cpu().numpy().astype(np.uint8)
                binary_mask = cv2.resize(binary_mask, (1280, 720))
                
                # Append the mask and its corresponding label to the list
                masks_ws_labels.append((SegmentInterface.segment_outlier(binary_mask), [x1, y1, x2, y2], int(label)))

        return masks_ws_labels
        
    @staticmethod
    def segment_outlier(segment):
        mask = cv2.connectedComponents(segment)[1]
        mask_1D = mask.reshape(-1,)
        delet_mask_1D = mask_1D[np.where(mask_1D > 0)]
        n = mode(delet_mask_1D , keepdims=True)[0][0]
        mask = np.where(mask!=n,0,mask) / n
        return mask.reshape(segment.shape).astype(np.uint8)