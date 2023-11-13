import numpy as np

# 50x50 grid of random qualities 1-4
# 4 = Safe
# 3 = Viable
# 2 = Unsafe
# 1 = Terrible

def quality():
    size = 10
    qual = np.zeros([size,size])

    for i in range(size):
        for j in range(size):
            qual[i,j] = np.random.randint(1,5)

    return qual, size

