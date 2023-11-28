import numpy as np

# 50x50 grid of random qualities 1-4
# 4 = Safe
# 3 = Viable
# 2 = Unsafe
# 1 = Terrible

def quality():
    size = 10
    qual = np.zeros([size,size])

    prob_1 = 0.1
    prob_2 = 0.3
    prob_3 = 0.3
    prob_4 = 1 - prob_1 - prob_2 - prob_3


    for i in range(size):
        for j in range(size):
            val = np.random.random()
            if val < prob_1:
                res = 1
            elif val < (prob_2+prob_1):
                res = 2
            elif val < (prob_3 + prob_2 + prob_1):
                res = 3
            else: res = 4

            qual[i,j] = res
            

    return qual, size

def LZ_gen():
    size = 10 #10x10 squares
    grid_size = 5 #5x5m
    mid = size//2
    lower_bnd = 1 - mid
    upper_bnd = mid - 1

    north = grid_size*np.random.randint(lower_bnd,upper_bnd)
    east = grid_size*np.random.randint(lower_bnd,upper_bnd)

    return north, east
