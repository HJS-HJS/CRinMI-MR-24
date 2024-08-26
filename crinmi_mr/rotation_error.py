import numpy as np

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
    print("T1", T1)
    print("T2", T2)
    
    v1 = T1[1, :3]
    v2 = T2[1, :3]

    print("v1", v1)
    print("v2", v1)

    # Normalize the vectors
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    
    # Compute the dot product
    dot_product = np.dot(v1_norm, v2_norm)
    
    # Clip the dot product to avoid numerical issues with arccos
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Compute the angle in radians
    angle = np.arccos(dot_product)
    print("angle: ", angle)
    return angle


