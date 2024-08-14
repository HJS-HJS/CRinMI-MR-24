import os
import re
import pickle
import numpy as np

class DataSaveInterface(object):
    def __init__(self, dir:str):
        self.save_dir = dir
        self.FILE_ZERO_PADDING_NUM = 4
        self.idx = self.data_num()

    def data_num(self):
        file_list = os.listdir(self.save_dir)
        return len(file_list)

    def save_data(self, *arg):
        name = ("/data_%0" + str(self.FILE_ZERO_PADDING_NUM) + 'd.p')%(self.idx)
        with open(self.save_dir + name, 'wb') as f:
            pickle.dump(arg, f)
        self.idx +=1
        return name
    
    def read_data(self, name):
        with open(self.save_dir + '/' + name, 'rb') as f:
            return pickle.load(f)
        