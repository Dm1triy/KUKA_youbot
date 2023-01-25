from numba import cuda
import numpy
# Create the data array - usually initialized some other way
data = numpy.ones(256)

# Set the number of threads in a block
threadsperblock = 1

# Calculate the number of thread blocks in the grid
blockspergrid = (data.size + (threadsperblock - 1)) // threadsperblock
print()

print(blockspergrid)
@cuda.jit
def my_kernel(io_array):
    """
    Code for kernel.
    """
    # code here

# Now start the kernel
my_kernel[blockspergrid, threadsperblock](data)

# Print the result
print(data)