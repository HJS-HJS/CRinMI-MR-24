import os
import copy
import yaml
import numpy as np
import matplotlib.pyplot as plt

import rospkg

# Set up workspace (adding z variation)
class CalibrationInterface(object):
    def __init__(self):
        mesh_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        config = self.load_yaml(mesh_dir + '/workspace.yaml')
        self.assemble_workspace = config["assemble"]["workspace"]
        self.assemble_offsets = config["assemble"]["offsets"]
        self.guide_workspace = config["guide"]["workspace"]
        self.guide_offsets = config["guide"]["offsets"]

    def load_yaml(self, file_path):
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
            return data
    
    def interpolate_tuple(self, tup1, tup2, fraction):
        return tuple(a + (b - a) * fraction for a, b in zip(tup1, tup2))
    
    def calculate_offset(self, point, assemble = True):
        if assemble:
            workspace = copy.deepcopy(self.assemble_workspace)
            offsets   = copy.deepcopy(self.assemble_offsets)
        else:
            workspace = copy.deepcopy(self.guide_workspace)
            offsets   = copy.deepcopy(self.guide_offsets)
        
        x, y, z = point
        # y linear interpolation
        if y < workspace['l_m'][1]:
            y_frac = (y - workspace['l_b'][1]) / (workspace['l_m'][1] - workspace['l_b'][1])
            offset_y_l = self.interpolate_tuple(offsets['l_b'], offsets['l_m'], y_frac)
            offset_y_r = self.interpolate_tuple(offsets['r_b'], offsets['r_m'], y_frac)
        else:
            y_frac = (y - workspace['l_m'][1]) / (workspace['l_t'][1] - workspace['l_m'][1])
            offset_y_l = self.interpolate_tuple(offsets['l_m'], offsets['l_t'], y_frac)
            offset_y_r = self.interpolate_tuple(offsets['r_m'], offsets['r_t'], y_frac)

        # x linear interpolation
        x_frac = (x - workspace['l_t'][0]) / (workspace['r_t'][0] - workspace['l_t'][0])
        offset_xy = self.interpolate_tuple(offset_y_l, offset_y_r, x_frac)

        # z linear interpolation
        z_frac = (z - workspace['l_b'][2]) / (workspace['l_t'][2] - workspace['l_b'][2])
        offset_xyz = self.interpolate_tuple(offset_xy, offset_xy, z_frac)

        correct_xyz = np.array([x + offset_xyz[0], y + offset_xyz[1], z])

        return offset_xyz, correct_xyz
    
    def calculate_point_interpolation(self, point, assemble = True):
        """ Calculate interpolation for an arbitrary point (x, y) within the 3x3 grid """
        
        if assemble:
            workspace = copy.deepcopy(self.assemble_workspace)
            offsets   = copy.deepcopy(self.assemble_offsets)
        else:
            workspace = copy.deepcopy(self.guide_workspace)
            offsets   = copy.deepcopy(self.guide_offsets)
        
        x, y = point

        # Determine the interpolation along y-axis
        if y > workspace['m_m'][1]:
            # For y < y_m, we use the top and bottom values
            if x > workspace['m_m'][0]:
                print("left bottom")
                # left bottom
                y_frac = (y - workspace['m_m'][1]) / (workspace['l_b'][1] - workspace['m_m'][1])
                offset_y_l = self.interpolate_tuple(offsets['m_m'], offsets['m_b'], y_frac)
                offset_y_r = self.interpolate_tuple(offsets['l_m'], offsets['l_b'], y_frac)
            else:
                # right bottom
                print("right bottom")
                y_frac = (y - workspace['m_m'][1]) / (workspace['r_b'][1] - workspace['m_m'][1])
                offset_y_l = self.interpolate_tuple(offsets['m_m'], offsets['m_b'], y_frac)
                offset_y_r = self.interpolate_tuple(offsets['r_m'], offsets['r_b'], y_frac)
        else:
            # For y >= y_m, we use the top and bottom values
            if x > workspace['m_m'][0]:
                # left top
                print("left top")
                y_frac = (y - workspace['l_t'][1]) / (workspace['m_m'][1] - workspace['l_t'][1])
                offset_y_l = self.interpolate_tuple(offsets['m_t'], offsets['m_m'], y_frac)
                offset_y_r = self.interpolate_tuple(offsets['l_t'], offsets['l_m'], y_frac)
            else:
                # right top
                print("right top")
                print(y)
                y_frac = (y - workspace['r_t'][1]) / (workspace['m_m'][1] - workspace['r_t'][1])
                print(y_frac)
                offset_y_l = self.interpolate_tuple(offsets['r_m'], offsets['r_t'], y_frac)
                offset_y_r = self.interpolate_tuple(offsets['m_m'], offsets['m_t'], y_frac)
        

        # Determine interpolation along x-axis
        x_frac = (x - workspace['r_t'][0]) / (workspace['l_t'][0] - workspace['r_t'][0])
        offset_xy = self.interpolate_tuple(offset_y_l, offset_y_r, x_frac)
        
        return offset_xy
    
    def update_yaml_workspace(self, base2cam, data_path, assemble = True):
    
        data = np.load(data_path)

        # 마커 좌표들을 읽어서 [x, y, z, 1] 형식의 nx4 matrix로 변환
        marker_names = [key for key in data.keys() if 'marker' in key]
        marker_positions = []

        for marker in marker_names:
            xyz = data[marker][:3]  # x, y, z 값만 추출
            xyz1 = np.append(xyz, 1)  # [x, y, z, 1] 형태로 변환
            marker_positions.append(xyz1)

        # nx4 matrix로 변환
        marker_positions = np.array(marker_positions)

        # Base 좌표계의 마커 좌표 계산 (nx4 matrix)
        marker_positions_in_base = (base2cam @ marker_positions.T).T

        base2marker = {}
        # Base 좌표계의 마커 좌표 출력
        for i, marker in enumerate(marker_names):
            print(marker)
            print(marker_positions_in_base[i])
            base2marker[marker] = marker_positions_in_base[i]

        if len(data) == 6:
            marker_id_mapping = {
                'l_t': "marker_40_pose",  # left top
                'l_m': "marker_30_pose",  # left middle
                'l_b': "marker_29_pose",  # left bottom
                'r_t': "marker_21_pose",  # right top
                'r_m': "marker_6_pose",  # right middle
                'r_b': "marker_4_pose"   # right bottom
            }
            marker_positions = {
                'l_t': base2marker[marker_id_mapping['l_t']][:3],
                'l_m': base2marker[marker_id_mapping['l_m']][:3],
                'l_b': base2marker[marker_id_mapping['l_b']][:3],
                'r_t': base2marker[marker_id_mapping['r_t']][:3],
                'r_m': base2marker[marker_id_mapping['r_m']][:3],
                'r_b': base2marker[marker_id_mapping['r_b']][:3]
            }
        elif len(data) == 9:
            marker_id_mapping = {
                'l_t': "marker_40_pose",  # left top
                'l_m': "marker_30_pose",  # left middle
                'l_b': "marker_29_pose",  # left bottom
                'm_t': "marker_21_pose",  # middle top
                'm_m': "marker_6_pose",  # middle middle
                'm_b': "marker_4_pose",  # middle bottom
                'r_t': "marker_5_pose",  # right top
                'r_m': "marker_18_pose",  # right middle
                'r_b': "marker_20_pose"   # right bottom
            }
            marker_positions = {
                'l_t': base2marker[marker_id_mapping['l_t']][:3],
                'l_m': base2marker[marker_id_mapping['l_m']][:3],
                'l_b': base2marker[marker_id_mapping['l_b']][:3],
                'm_t': base2marker[marker_id_mapping['m_t']][:3],
                'm_m': base2marker[marker_id_mapping['m_m']][:3],
                'm_b': base2marker[marker_id_mapping['m_b']][:3],
                'r_t': base2marker[marker_id_mapping['r_t']][:3],
                'r_m': base2marker[marker_id_mapping['r_m']][:3],
                'r_b': base2marker[marker_id_mapping['r_b']][:3]
            }
        else:
            print("ERROR")
            return

        print("WORKSPACE")
        if assemble:
            print("assemble:")
        else:
            print("guide:")
        print("  workspace:")
        for key, value in marker_positions.items():
            print(f"    {key}: [{', '.join(f'{v:.8f}' for v in value)}]")
        print("\n")

    def update_yaml_offsets(self, file_path, assemble = True):
        # YAML 파일 읽기
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        # workspace와 measure 데이터 추출
        if assemble:        
            workspace = yaml_data['assemble']['workspace']
            measure = yaml_data['assemble']['measure']
        else:
            workspace = yaml_data['guide']['workspace']
            measure = yaml_data['guide']['measure']

        # offsets 계산
        offsets = {key: [measure[key][i] - workspace[key][i] for i in range(3)] for key in workspace}

        # YAML 데이터 업데이트
        yaml_data['assemble']['offsets'] = offsets

        print("OFFSETS")
        print("  offsets:")
        for key, value in offsets.items():
            print(f"    {key}: [{', '.join(f'{v:.8f}' for v in value)}]")


if __name__ == '__main__':
    cal = CalibrationInterface()

    x = np.linspace(0.18, 0.4, 100)
    y = np.linspace(-0.86, -0.5, 100)
    X, Y = np.meshgrid(x, y)

    # Calculate offsets
    Z_offset = np.zeros_like(X)

    # 각 점에서의 오프셋 계산
    Z_offset = np.zeros((100, 100, 3))
    for i in range(100):
        for j in range(100):
            Z_offset[i, j] = cal.calculate_offset((X[i, j], Y[i, j], 0), assemble=True)[0]

    # 오프셋 각 성분 추출
    Z_offset_x = Z_offset[:, :, 0]
    Z_offset_y = Z_offset[:, :, 1]
    Z_offset_z = Z_offset[:, :, 2]

    # 결과를 2D로 플롯
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    # X 오프셋 플롯
    im1 = axes[0].imshow(Z_offset_x, extent=(X.min(), X.max(), Y.min(), Y.max()), origin='lower', cmap='viridis')
    axes[0].set_title('X Offset')
    axes[0].set_xlabel('X')
    axes[0].set_ylabel('Y')
    fig.colorbar(im1, ax=axes[0], orientation='vertical')

    # Y 오프셋 플롯
    im2 = axes[1].imshow(Z_offset_y, extent=(X.min(), X.max(), Y.min(), Y.max()), origin='lower', cmap='viridis')
    axes[1].set_title('Y Offset')
    axes[1].set_xlabel('X')
    axes[1].set_ylabel('Y')
    fig.colorbar(im2, ax=axes[1], orientation='vertical')

    # Z 오프셋 플롯
    im3 = axes[2].imshow(Z_offset_z, extent=(X.min(), X.max(), Y.min(), Y.max()), origin='lower', cmap='viridis')
    axes[2].set_title('Z Offset')
    axes[2].set_xlabel('X')
    axes[2].set_ylabel('Y')
    fig.colorbar(im3, ax=axes[2], orientation='vertical')

    plt.tight_layout()
    plt.show()