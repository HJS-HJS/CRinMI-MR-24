import os
import cv2
import torch
from PIL import Image
from ultralytics import YOLO

start_path = os.getcwd()
model_path = os.path.join(start_path, "best_1280.pt")
model = YOLO(model_path)

source = os.path.join(start_path, "data/test/images/data_0052-p_image_jpg.rf.9b97463173f4f8b4fad245fa9d6bce4b.jpg")
# source = os.path.join(start_path, "1_image_jpg.rf.34c502ec5092bfe07b368b7e95a00fb2.jpg")
# source = os.path.join(start_path, "segment_rgb.png")
print(source)
results = model.predict(source, conf=0.5)

im_array = results[0].plot()  # plot a BGR numpy array of predictions
im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image
im.show()  # show image

# for result in results:
#     # get array results
#     print("work?")
#     masks = result.masks.data
#     boxes = result.boxes.data
#     # extract classes
#     clss = boxes[:, 5]
#     # get indices of results where class is 0 (people in COCO)
#     people_indices = torch.where(clss == 2)

#     # use these indices to extract the relevant masks
#     people_masks = masks[people_indices]

#     # scale for visualizing results
#     people_mask = torch.any(people_masks, dim=0).int() * 255

#     # save to file
#     cv2.imwrite(start_path + '/seg.jpg', people_mask.cpu().numpy())