import numpy as np

slicer = (slice(0, 4), slice(0, 4))
a = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]])
print(a[slicer[0], slicer[1]])
