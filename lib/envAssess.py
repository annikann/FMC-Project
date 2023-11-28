# Assess landing environment and determine when safe to land

import numpy as np
import random

grid_dim = 5.
drone_pn = -15.
drone_pe = 0.

def assessLand(groundFaceColors, grid_dim, drone_pn, des_pn, drone_pe, des_pe):
    grid_size = len(groundFaceColors)/2
    safety_map = np.reshape(groundFaceColors, (int(grid_size/grid_dim), int(grid_size/grid_dim)))
    safety_map[safety_map=='tab:red'] = 0
    safety_map[safety_map=='tab:orange'] = 1
    safety_map[safety_map=='y'] = 2
    safety_map[safety_map=='tab:green'] = 3
    safety_map = safety_map.astype(int)
    land_zone = []

    east_vals = np.arange(-20, 25, 5)
    north_vals = np.arange(-20, 25, 5)

    # Gets landing zone squares based on desired end spot
    for i in range(len(east_vals)):
        if east_vals[i] == des_pe:
            col1 = i
            col2 = i + 1
    for i in range(len(north_vals)):
        if north_vals[i] == -des_pn:
            row1 = i
            row2 = i + 1

    land_zone.append(safety_map[row1, col1]) # upper left
    land_zone.append(safety_map[row1, col2]) # upper right
    land_zone.append(safety_map[row2, col1]) # lower left
    land_zone.append(safety_map[row2, col2]) # lower right
    # print(col1, col2)
    # print(row1, row2)
    # print(land_zone)

    # Nest these statements under a check for the actual positions reaching
    # within a certain range of the desired.
    if 0. in land_zone:
        test = str('Unsafe to land, find new landing zone.')
        result = False
    else:
        land = np.average(land_zone)
        if land > 1.:
            test = str('Safe to land.')
            result = True
        else:
            test = str('Unsafe to land, find new landing zone.')
            result = False
    
    return test, result

# test = assessLand(groundFaceColors, grid_dim, drone_pn, drone_pe)
# print(test)