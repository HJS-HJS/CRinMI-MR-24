import numpy as np
import tf.transformations as tf

def pose2matrix(pose, mm=False):
    if len(pose) != 6:
        raise Exception("[pose2matrix] Size of pose array is not 6")
    if mm:
        pose[0:3] = np.array(pose[0:3]) / 1000
    else:
        pose[0:3] = np.array(pose[0:3])

    pose[3:6] = np.deg2rad(np.array(pose[3:6]))
    matrix = tf.euler_matrix(pose[3], pose[4], pose[5])
    matrix[:3,3] = pose[0:3]
    return matrix

def rotation(x, y, z, axes = 'sxyz'):
    x = np.deg2rad(x)
    y = np.deg2rad(y)
    z = np.deg2rad(z)

    return tf.euler_matrix(x, y, z, axes=axes)[:3,:3]

def marker_angle(marker_list):
    marker36 = np.array([0.01044702,  0.09468093])
    marker30 = np.array([0.11187456, -0.10777453])
    marker4  = np.array([1.06086447e-02, -1.11210563e-01])
    marker5  = np.array([0.11249995,  0.0950142 ])

    marker_set1 = np.array(marker4-marker36)
    marker_set2 = np.array(marker30-marker5)

    angle1 = np.rad2deg(np.arctan2(marker_set1[1], marker_set1[0]))
    angle2 = np.rad2deg(np.arctan2(marker_set2[1], marker_set2[0]))

    print(angle1)
    print(angle2)