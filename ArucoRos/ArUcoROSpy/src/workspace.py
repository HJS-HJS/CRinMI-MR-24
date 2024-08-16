import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
from scipy.spatial.transform import Rotation as R

def quaternion_to_rotation_matrix(quaternion):
    """쿼터니언을 회전 행렬로 변환합니다."""
    r = R.from_quat(quaternion)
    return r.as_matrix()

def transform_marker_pose(tvec, quaternion, T_camera_to_ee, T_ee_to_base):
    """
    마커의 포즈를 로봇 베이스 좌표계로 변환합니다.

    :param tvec: 카메라 좌표계에서의 마커의 변위 벡터 (3x1)
    :param quaternion: 카메라 좌표계에서의 마커의 회전 쿼터니언 (4x1)
    :param T_camera_to_ee: 카메라 좌표계에서 엔드 이펙터 좌표계로의 변환 행렬 (4x4)
    :param T_ee_to_base: 엔드 이펙터 좌표계에서 로봇 베이스 좌표계로의 변환 행렬 (4x4)
    :return: 로봇 베이스 좌표계에서의 마커의 변위 벡터 (3x1) 및 회전 행렬 (3x3)
    """
    # 3x1 tvec을 4x1로 변환
    tvec_homogeneous = np.append(tvec, 1).reshape(4, 1)

    # 카메라 좌표계에서 엔드 이펙터 좌표계로 변환
    tvec_ee = np.dot(T_camera_to_ee, tvec_homogeneous)

    # 엔드 이펙터 좌표계에서 로봇 베이스 좌표계로 변환
    tvec_base = np.dot(T_ee_to_base, tvec_ee)

    # 쿼터니언을 회전 행렬로 변환
    R_camera_to_marker = quaternion_to_rotation_matrix(quaternion)
    
    # 카메라 좌표계에서 엔드 이펙터 좌표계로 변환
    R_ee_to_camera = T_camera_to_ee[:3, :3]
    R_ee_to_marker = np.dot(R_ee_to_camera, R_camera_to_marker)
    
    # 엔드 이펙터 좌표계에서 로봇 베이스 좌표계로 변환
    R_base_to_ee = T_ee_to_base[:3, :3]
    R_base_to_marker = np.dot(R_base_to_ee, R_ee_to_marker)

    return tvec_base[:3], R_base_to_marker

# 변환 행렬 정의
T_camera_to_ee = np.array([[-1., 0., 0., 29.604876295725482],
                           [ 0., 1., 0., 69.03497489293414],
                           [ 0., 0., -1., 142.4168182373045],
                           [ 0., 0., 0., 1. ]], dtype=np.float32)

T_ee_to_base = np.eye(4, dtype=np.float32)

# 임의의 직육면체 꼭짓점 (카메라 좌표계에서 측정된 값)
vertices_camera = np.array([
    [0.1, 0.2, 0.3],
    [0.1, 0.2, 0.5],
    [0.1, 0.4, 0.3],
    [0.1, 0.4, 0.5],
    [0.3, 0.2, 0.3],
    [0.3, 0.2, 0.5],
    [0.3, 0.4, 0.3],
    [0.3, 0.4, 0.5]
])

quaternion_camera = [0, 0, 0, 1]  # 단위 쿼터니언 (회전 없음)

# 로봇 베이스 좌표계로 변환된 꼭짓점
vertices_base = []
for vertex in vertices_camera:
    tvec_base, R_base = transform_marker_pose(vertex, quaternion_camera, T_camera_to_ee, T_ee_to_base)
    vertices_base.append(tvec_base)

vertices_base = np.array(vertices_base)

# 시각화 함수 정의
def plot_coordinate_system(ax, T, label, color):
    origin = T[:3, 3]
    x_axis = T[:3, 0]
    y_axis = T[:3, 1]
    z_axis = T[:3, 2]

    ax.quiver(*origin, *x_axis, color=color[0], length=0.1, normalize=True)
    ax.quiver(*origin, *y_axis, color=color[1], length=0.1, normalize=True)
    ax.quiver(*origin, *z_axis, color=color[2], length=0.1, normalize=True)
    ax.text(*origin, label, color='black')

# 시각화
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 카메라 좌표계 (원점)
T_camera = np.eye(4, dtype=np.float32)
plot_coordinate_system(ax, T_camera, 'Camera', ['r', 'g', 'b'])

# 엔드 이펙터 좌표계
plot_coordinate_system(ax, T_camera_to_ee, 'End-Effector', ['r', 'g', 'b'])

# 로봇 베이스 좌표계
plot_coordinate_system(ax, T_ee_to_base, 'Base', ['r', 'g', 'b'])

# 변환된 마커 꼭짓점 (로봇 베이스 좌표계에서)
ax.scatter(vertices_base[:, 0], vertices_base[:, 1], vertices_base[:, 2], color='magenta', s=100)
for i, vertex in enumerate(vertices_base):
    ax.text(*vertex, f'P{i+1}', color='magenta')

# 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Coordinate Systems and Transformed Marker Vertices')
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.5])
ax.view_init(elev=20, azim=30)
plt.show()
