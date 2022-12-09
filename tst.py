import scipy
import numpy as np

# robot_radius = 7
# n_mask = scipy.ndimage.generate_binary_structure(2, 1)
# m_mask = scipy.ndimage.generate_binary_structure(2, 2)
# print(n_mask)
# robot_mask = np.zeros((2 * robot_radius, 2 * robot_radius))
# robot_mask[robot_radius][robot_radius] = 1
#
# # 1 var
# robot_mask_1 = scipy.ndimage.binary_dilation(robot_mask, structure=n_mask).astype(n_mask.dtype)
# for i in range(robot_radius-1):
#     robot_mask_1 = scipy.ndimage.binary_dilation(robot_mask_1, structure=n_mask).astype(n_mask.dtype)
#
# # 2 var
# robot_mask_2 = scipy.ndimage.binary_dilation(robot_mask, structure=n_mask).astype(n_mask.dtype)
# for i in range(robot_radius-1):
#     robot_mask_2 = scipy.ndimage.binary_dilation(robot_mask_1, structure=m_mask).astype(m_mask.dtype)
#
# pass

map_radius = 7
bool_map = np.zeros((2 * map_radius, 2 * map_radius))
bool_map[map_radius][map_radius] = 1
bool_map[map_radius+1][map_radius+1] = 1

for i in range(0, 6):
    for j in range(1, 7):
        bool_map[i][j] = 1

for i in range (10, 13):
    for j in range (10, 13):
        bool_map[i][j] = 1

print(bool_map)

#filter = np.ones((1, 1))
# filter = scipy.ndimage.generate_binary_structure(2, 1)
# filter[1, 2] = 0
# filter[1, 0] = 0
# filter[2, 1] = 0
filter = np.ones((1, 3))
filter[0][0] = 0
print(filter)
bool_map = scipy.ndimage.binary_erosion(bool_map, structure=filter, border_value=1).astype(bool_map.dtype)
bool_map = scipy.ndimage.binary_dilation(bool_map, structure=filter).astype(bool_map.dtype)
print(bool_map)