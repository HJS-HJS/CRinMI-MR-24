#!/usr/bin/env python3
import rospy
import math
import tf
import geometry_msgs.msg
import pickle
import os

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    tf_listener = tf.TransformListener()
    if not rospy.has_param('~img_id'):
        rospy.logerr('Please specify image ID by rosparam: img_id:=0')
        exit()
    img_id = int(rospy.get_param('~img_id'))
    rospy.loginfo(f'Image ID: {img_id}')
    image_name = 'image' + str(img_id)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if tf_listener.canTransform(
                target_frame="/rgb_camera_link", source_frame="mobile_robot", time=rospy.Time(0)):
            # TF 변환이 가능한지 검사. 불가능 할 때, lookupTransform을 호출할 경우 예외 발생
            # TF 변환시 시간은 rospy.Time.now()가 아니라 Time(0)을 써야한다!!

            tf_matrix = tf_listener.lookupTransform(
                target_frame="/rgb_camera_link", source_frame="mobile_robot", time=rospy.Time(0))
            # 변환 행렬 리턴. 리턴 값은 ([x, y, z], [x, y, z, w])

            rospy.loginfo(tf_matrix)
            fname = os.path.join('/home/kkw0418/query/pose', f'{image_name}_pose.txt')
            with open(fname, 'w') as f:
                print(tf_matrix, file=f)
            

        else:
            rospy.logwarn("Cannot lookup transform between world and home")

        r.sleep()