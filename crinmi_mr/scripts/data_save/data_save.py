import os
import pickle

class DataSaveInterface(object):
    def __init__(self, dir:str):
        self.save_dir = dir
        self.FILE_ZERO_PADDING_NUM = 4
        self.idx = self.data_num()

    def data_num(self):
        file_list = os.listdir(self.save_dir)
        return len(file_list)

    def save_data(self, *arg):
        name = ("/depth_%0" + str(self.FILE_ZERO_PADDING_NUM) + 'd.p')%(self.idx)
        with open(self.save_dir + name, 'wb') as f:
            pickle.dump(arg, f)
        self.idx +=1
        return name
    
    def read_data(self, name):
        with open(self.save_dir + '/' + name, 'rb') as f:
            return pickle.load(f)

    def get_data(self):
        import numpy as np
        from cv_bridge import CvBridge
        import cv2
        cv_bridge = CvBridge()
        file_list = os.listdir(self.save_dir)
        for file in file_list:
            data = self.read_data(file)
            cv2.imwrite(self.save_dir + '/' + file + '_image.jpg', cv_bridge.imgmsg_to_cv2(data[0], "bgr8"))
        # data = self.read_data(file_list[0])
        # cv2.imwrite(self.save_dir + '/' + file_list[20] + '_image.jpg', cv_bridge.imgmsg_to_cv2(data[0], "bgr8"))


if __name__ == '__main__':
    module = DataSaveInterface("/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/data")
    module.get_data()
        