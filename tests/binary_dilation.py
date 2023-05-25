import numpy as np
from scipy import ndimage

weight = np.random.uniform(1, 1.5, (10, 10))
weight[3:7, 3:7] = 2
weight[5:8, 2] = 2
new = np.where(weight > 1.9, 1, 0)
print(new)

struct = ndimage.generate_binary_structure(2, 2)
print("struct1:\n", struct, '\n')

struct2 = [[False, True, False],
           [False, True, False],
           [False, True, False]]

struct2 = np.zeros((3, 3)).astype(bool)
struct2[1, 1:] = True
print(struct2)

new = ndimage.binary_dilation(new, structure=struct2).astype(new.dtype)
print(new)

new = ndimage.binary_erosion(new, structure=struct2).astype(new.dtype)
print(new)


# new = ndimage.binary_dilation(new, structure=struct2).astype(new.dtype)
# print(new)


# new = ndimage.binary_dilation(new, structure=struct).astype(new.dtype)
# print(new)

new = (new-1)*(-1)
# print(new)

# new = np.where(new == 1, 1.6, 1)
# print(new)
