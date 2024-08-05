#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class Test(object):
    
    def __init__(self):
        # param
        rospy.loginfo('Running')
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    
    rospy.spin()
