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

def pose2matrixz(pose, mm=False):
    if len(pose) != 6:
        raise Exception("[pose2matrix] Size of pose array is not 6")
    if mm:
        pose[0:3] = np.array(pose[0:3]) / 1000
    else:
        pose[0:3] = np.array(pose[0:3])

    theta = pose[5]
    pose[3:6] = np.deg2rad(np.array(pose[3:6]))
    matrix = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta),  np.cos(theta), 0, 0],
                        [       0,         0, 1, 0],
                        [       0,         0, 0, 1]])
    matrix[:3,3] = pose[0:3]
    return matrix

def rotation(x, y, z, axes = 'sxyz'):
    x = np.deg2rad(x)
    y = np.deg2rad(y)
    z = np.deg2rad(z)

    return tf.euler_matrix(x, y, z, axes=axes)[:3,:3]

def marker_angle(marker_list):
    marker36 = np.array([])
    marker30 = np.array([])
    marker4  = np.array([])
    marker5  = np.array([])

    marker_set1 = np.array(marker36-marker30)
    marker_set2 = np.array(marker4-marker5)

    angle1 = np.rad2deg(np.arctan2(marker_set1[1], marker_set1[0]))
    angle2 = np.rad2deg(np.arctan2(marker_set2[1], marker_set2[0]))

    print(angle1)
    print(angle2)

def rotation_matrix_error(T1, T2):
    """
    Compute the rotation matrix error between two rotation matrices.
    
    Parameters:
    R1 (np.ndarray): The first rotation matrix.
    R2 (np.ndarray): The second rotation matrix.
    
    Returns:
    float: The rotation angle representing the error in radians.
    """
    # Z axis of rotation matrix
    v1 = T1[2, :3]
    v2 = T2[2, :3]
    # print("v1", v1)
    # print("v2", v1)

    # Normalize the vectors
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    
    # Compute the dot product
    dot_product = np.dot(v1_norm, v2_norm)
    
    # Clip the dot product to avoid numerical issues with arccos
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Compute the angle in radians
    angle = np.arccos(dot_product)
    
    return angle
    