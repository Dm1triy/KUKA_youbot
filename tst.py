import scipy
import numpy as np

robot_radius = 7
n_mask = scipy.ndimage.generate_binary_structure(2, 1)
m_mask = scipy.ndimage.generate_binary_structure(2, 2)
print(n_mask)
robot_mask = np.zeros((2 * robot_radius, 2 * robot_radius))
robot_mask[robot_radius][robot_radius] = 1

# 1 var
robot_mask_1 = scipy.ndimage.binary_dilation(robot_mask, structure=n_mask).astype(n_mask.dtype)
for i in range(robot_radius-1):
    robot_mask_1 = scipy.ndimage.binary_dilation(robot_mask_1, structure=n_mask).astype(n_mask.dtype)

# 2 var
robot_mask_2 = scipy.ndimage.binary_dilation(robot_mask, structure=n_mask).astype(n_mask.dtype)
for i in range(robot_radius-1):
    robot_mask_2 = scipy.ndimage.binary_dilation(robot_mask_1, structure=m_mask).astype(m_mask.dtype)

pass