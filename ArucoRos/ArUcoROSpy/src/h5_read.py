import cv2
import datetime as dt
import h5py
import numpy as np
import os
import fnmatch

start = dt.datetime.now()

#이미지가있는 폴더 경로

PATH = os.path.abspath(os.path.join('..', 'input'))

SOURCE_IMAGES = os.path.join(PATH, "images")

#h5파일 읽기 시작..

f = h5py.File('/home/kkw0418/Downloads/extractor+matcher/superpoint_max+NN-mutual/outputs/demo/features.h5', 'r')

dataset_names = list(f.keys())

for dataset_name in dataset_names:
    print(dataset_name)

print("999")

data = f['mapping']

dataset_names = list(data.keys())

for dataset_name in dataset_names:
    print(dataset_name)
print("84125")

im00 = data["image000.jpg"]

dataset_names = list(im00.keys())
for dataset_name in dataset_names:
    print(dataset_name)
    
print("15125")
print(im00["keypoints"])