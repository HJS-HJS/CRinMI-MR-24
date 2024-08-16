import os
import pickle

 
### 피클 파일 불러오기 ###
with open("/home/kkw0418/Desktop/pcd/image0.pkl","rb") as fr:
    data = pickle.load(fr)

print(data.fields)