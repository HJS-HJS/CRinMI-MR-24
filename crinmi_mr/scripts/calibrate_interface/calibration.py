import copy
import yaml
import numpy as np
import matplotlib.pyplot as plt

# Set up workspace (adding z variation)
class CalibrationInterface(object):
    def __init__(self):
        config = self.load_yaml('/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/config/workspace.yaml')
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

        correct_xyz = np.array([x + offset_xyz[0], y + offset_xyz[1], z + offset_xyz[2]])

        return offset_xyz, correct_xyz

# # Example code
# cal = CalibrationInterface()

# points = [
#     [0.31241449, -0.70576825, 0.00206379, 1.0], #36
#     [3.62813190e-01, -8.53884383e-01, 2.37018143e-04, 1.0], #37
#     [0.21649703, -0.76158226, 0.00398036, 1.0], #30
#     [0.30494156, -0.57011973, 0.01109796, 1.0], #21
#     [0.2144644, -0.62775603, 0.00193548, 1.0] #4
# ]

# for point in points:
#     x, y, z = point[:3]  # 앞의 x, y, z 값만 추출
#     offset = cal.calculate_offset(x, y, z)  # 함수에 x, y, z 값을 넣음
#     print(offset[1])

# for i in range(X.shape[0]):
#     for j in range(X.shape[1]):
#         offset = cal.calculate_offset(X[i, j], Y[i, j], Z[i, j])
#         # offset = cal.calculate_offset(X[i, j], Y[i, j], Z[i])
#         Zx[i, j] = offset[0]
#         Zy[i, j] = offset[1]
#         Zz[i, j] = offset[2]

# # Plot the results
# fig, axs = plt.subplots(1, 3, figsize=(18, 6))

# # Plot X offsets
# c1 = axs[0].contourf(X, Y, Zx, cmap='viridis')
# axs[0].set_title('Offset X')
# fig.colorbar(c1, ax=axs[0])

# # Plot Y offsets
# c2 = axs[1].contourf(X, Y, Zy, cmap='plasma')
# axs[1].set_title('Offset Y')
# fig.colorbar(c2, ax=axs[1])

# # Plot Z offsets
# c3 = axs[2].contourf(X, Y, Zz, cmap='inferno')
# axs[2].set_title('Offset Z')
# fig.colorbar(c3, ax=axs[2])

# plt.tight_layout()
# plt.show()
