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
