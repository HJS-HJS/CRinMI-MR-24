import yaml
import numpy as np
import matplotlib.pyplot as plt

# Set up self.workspace (adding z variation)
class calibration_interface(object):
    def __init__(self):
        config = self.load_yaml('/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/config/workspace.yaml')
        self.workspace = config["workspace"]
        self.offsets = config["offsets"]

    def load_yaml(self, file_path):
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
            return data
    
    def interpolate_tuple(self, tup1, tup2, fraction):
        return tuple(a + (b - a) * fraction for a, b in zip(tup1, tup2))
    
    def calculate_offset(self, x, y, z):
        # y linear interpolation
        if y < self.workspace['l_m'][1]:
            y_frac = (y - self.workspace['l_b'][1]) / (self.workspace['l_m'][1] - self.workspace['l_b'][1])
            offset_y_l = self.interpolate_tuple(self.offsets['l_b'], self.offsets['l_m'], y_frac)
            offset_y_r = self.interpolate_tuple(self.offsets['r_b'], self.offsets['r_m'], y_frac)
        else:
            y_frac = (y - self.workspace['l_m'][1]) / (self.workspace['l_t'][1] - self.workspace['l_m'][1])
            offset_y_l = self.interpolate_tuple(self.offsets['l_m'], self.offsets['l_t'], y_frac)
            offset_y_r = self.interpolate_tuple(self.offsets['r_m'], self.offsets['r_t'], y_frac)

        # x linear interpolation
        x_frac = (x - self.workspace['l_t'][0]) / (self.workspace['r_t'][0] - self.workspace['l_t'][0])
        offset_xy = self.interpolate_tuple(offset_y_l, offset_y_r, x_frac)

        # z linear interpolation
        z_frac = (z - self.workspace['l_b'][2]) / (self.workspace['l_t'][2] - self.workspace['l_b'][2])
        offset_xyz = self.interpolate_tuple(offset_xy, offset_xy, z_frac)

        correct_xyz = np.array([x + offset_xyz[0], y + offset_xyz[1], z + offset_xyz[2]])

        return offset_xyz, correct_xyz

# Example code
cal = calibration_interface()
# Generate a grid of points in the plane
x_vals = np.linspace(0.19, 0.38, 100)
y_vals = np.linspace(-0.87, -0.52, 100)
X, Y = np.meshgrid(x_vals, y_vals)

# Assume a linear z variation across the plane
Z = 2 * (X - Y) / 10.0
print(X.shape)
print(Z)
print(Z.shape)
Z = np.linspace(0, 0.1, 100)
Z = np.meshgrid(Z,Z)
print(Z)
print(Z.shape)

# Calculate the self.offsets for the grid points
Zx = np.zeros_like(X)
Zy = np.zeros_like(Y)
Zz = np.zeros_like(Y)

for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        offset = cal.calculate_offset(X[i, j], Y[i, j], Z[i, j])
        # offset = cal.calculate_offset(X[i, j], Y[i, j], Z[i])
        Zx[i, j] = offset[0]
        Zy[i, j] = offset[1]
        Zz[i, j] = offset[2]

# Plot the results
fig, axs = plt.subplots(1, 3, figsize=(18, 6))

# Plot X self.offsets
c1 = axs[0].contourf(X, Y, Zx, cmap='viridis')
axs[0].set_title('Offset X')
fig.colorbar(c1, ax=axs[0])

# Plot Y self.offsets
c2 = axs[1].contourf(X, Y, Zy, cmap='plasma')
axs[1].set_title('Offset Y')
fig.colorbar(c2, ax=axs[1])

# Plot Z self.offsets
c3 = axs[2].contourf(X, Y, Zz, cmap='inferno')
axs[2].set_title('Offset Z')
fig.colorbar(c3, ax=axs[2])

plt.tight_layout()
plt.show()
