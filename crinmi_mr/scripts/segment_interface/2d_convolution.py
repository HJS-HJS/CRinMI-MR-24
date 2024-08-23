from yolo_segment_interface import SegmentInterface
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import open3d as o3d

current_dir = os.path.dirname(os.path.realpath(__file__))

def depth2pcd(depth: np.array, extr: np.array = np.eye(4)) -> np.array:
        """Convert depth image to pointcloud data.

        Args:
            depth_image (np.array): (H, W) depth image to convert.
            intr (np.array): (3, 3) camera intrinsic matrix.
            extr (np.array): (3, 3) camera extrinsic matrix.

        Returns:
            np.array: (N, 3) pointcloud data array converted from depth image
        """

        depth_cam_intr = np.array([908.5108032226562, 0.0, 643.5980834960938, 0.0, 906.0470581054688, 349.9342041015625, 0.0, 0.0, 1.0])
        depth_cam_intr = depth_cam_intr.reshape((3, 3))
        print(depth_cam_intr.shape)

        height, width = depth.shape
        row_indices = np.arange(height)
        col_indices = np.arange(width)
        pixel_grid = np.meshgrid(col_indices, row_indices)
        pixels = np.c_[pixel_grid[0].flatten(), pixel_grid[1].flatten()].T
        pixels_homog = np.r_[pixels, np.ones([1, pixels.shape[1]])]
        print(pixels_homog.shape)
        depth_arr = np.tile(depth.flatten(), [3, 1])
        print(depth_arr.shape)
        point_cloud = depth_arr * np.linalg.inv(depth_cam_intr).dot(pixels_homog)
        point_cloud = point_cloud.transpose()

        return (np.matmul(extr[:3,:3], point_cloud[:,:3].T) + extr[:3,3].reshape(3,1)).T

def rotate_image(image, angle, center):
    """
    주어진 각도(angle)로 이미지를 회전시키는 함수.
    """
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_image = cv2.warpAffine(
        image, rotation_matrix, (image.shape[1], image.shape[0])
    )
    return rotated_image


def find_rotation_angle(image1, image2, center):
    """
    두 이미지의 합성곱(Convolution) 값을 비교하여 최댓값이 나오는 회전 각도를 찾는 함수.
    """
    max_correlation = -float("inf")
    best_angle = 0

    cor_list = []
    # 모든 각도에 대해 회전하며 Convolution 값 비교
    for angle in range(0, 360):
        rotated_image = rotate_image(image2, angle, center)
        # print("rotated_image", rotated_image.shape)
        # print("rotated_image", rotated_image.dtype)

        # # Plot
        res = cv2.bitwise_and(rotated_image, image1)

        # plt.figure(figsize=(10, 5))
        # plt.subplot(1, 2, 1)
        # plt.title('Original Mask')
        # plt.imshow(res, cmap='gray')
        # plt.show()

        # 이미지 간의 합성곱 계산 (정규화 포함)
        correlation = np.sum(res)
        cor_list.append(correlation)
        # 최댓값을 찾고 그에 해당하는 각도 저장
        if correlation > max_correlation:
            max_correlation = correlation
            best_angle = angle
    
    return best_angle, max_correlation, cor_list

def point_cloud_to_image(point_cloud, image_size):
    # 이미지 크기 (height, width)
    height, width = image_size
    
    # 이미지의 중심 좌표
    center_x, center_y = width // 2, height // 2
    
    # 빈 이미지 생성 (검은색)
    image = np.zeros((height, width), dtype=np.uint8)
    point_cloud = 1000*point_cloud
    # print(np.min(point_cloud[:, 0].astype(int)))
    # Point Cloud의 x, y 좌표를 픽셀 좌표로 변환
    # x, y 좌표의 0, 0 위치를 이미지의 중심으로 설정
    x_coords = np.clip(center_x + (point_cloud[:, 0]).astype(int), 0, width - 1)
    y_coords = np.clip(center_y - (point_cloud[:, 1]).astype(int), 0, height - 1)

    # 해당 좌표를 하얀색으로 설정
    image[y_coords, x_coords] = 255
    print("Return")
    return image

def save_image(image, filename):
    # 이미지 저장
    plt.imsave(filename, image, cmap='gray')

def save_ground_truth(rgb, depth, gt = False):

    # Run Yolo
    yolo = SegmentInterface()
    yolo.run(rgb)
    masks = yolo.img2SegmentMask()

    for mask, ws, label in masks:

        filtered_depth = depth * mask
        # Generate Point Cloud
        pcd_points = depth2pcd(depth=filtered_depth)
        # Open3D 포인트 클라우드 객체 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_points)
        pcd = pcd.voxel_down_sample(voxel_size=0.001)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
        pcd = pcd.select_by_index(ind)
        pcd.translate([-pcd.get_center()[0],-pcd.get_center()[1], 0])

        # Reprojection
        pcd_points = np.asarray(pcd.points)
        image = point_cloud_to_image(pcd_points, (720,1280))
        
        # Save
        if gt:
            save_image(image, "/home/kkwkmw/Desktop/mine/segment_interface/ground_truth/" + str(label) + ".png")
        else:
            save_image(image, "/home/kkwkmw/Desktop/mine/segment_interface/target/" + str(label) + ".png")

if __name__ == "__main__":
    # # load image
    # rgb = cv2.imread("/home/kkwkmw/Desktop/mine/data/0063color.png")
    # depth = np.load("/home/kkwkmw/Desktop/mine/data/0063depth.npy")
    
    # # Create ground truth
    # save_ground_truth(rgb, depth, False)

    # 폴더 경로 설정
    gt_folder = '/home/kkwkmw/Desktop/mine/segment_interface/ground_truth'  # 첫 번째 폴더
    target_folder = '/home/kkwkmw/Desktop/mine/segment_interface/target'  # 두 번째 폴더

    # 폴더 A에서 모든 파일을 불러옴
    targets = os.listdir(target_folder)
    center_x = 640
    center_y = 360
    # 각 파일에 대해 동일한 이름의 파일을 폴더 B에서 찾아 처리
    for file_name in targets:
        target_file = os.path.join(target_folder, file_name)
        
        # 폴더 B에서 동일한 이름의 파일을 찾음
        ground_truth_file = os.path.join(gt_folder, file_name)
        
        # 폴더 B에 동일한 파일이 있는지 확인
        if os.path.exists(gt_folder):
            # 이미지를 불러옴
            target = cv2.imread(target_file)
            ground_truth = cv2.imread(ground_truth_file)
            
            # Convolution
            best_angle, max_correlation, cor_list = find_rotation_angle(
                ground_truth, target, (center_x, center_y)
            )

            print(f"추정된 회전 각도: {best_angle} degrees")
            print(f"최대 합성곱 값: {max_correlation}")

            # 결과 시각화
            plt.figure(figsize=(10, 5))
            plt.subplot(1, 2, 1)
            plt.title("Original Image 1")
            plt.imshow(ground_truth, cmap="gray")
            plt.subplot(1, 2, 2)
            plt.title(f"Rotated Image 2 by {-15} degrees")
            plt.imshow(target, cmap="gray")
            plt.show()

            # 그래프 플롯
            x = list(range(360))  # angle
            plt.figure(figsize=(10, 5))
            plt.plot(x, cor_list, marker="o")

            # 그래프 제목 및 축 라벨 설정
            plt.title("Correlation val")
            plt.xlabel("Angle (degrees)")
            plt.ylabel("Correlation")

            # 그래프 보여주기
            plt.grid(True)
            plt.show()
        else:
            print(f"No matching file found in folder B for {file_name}")