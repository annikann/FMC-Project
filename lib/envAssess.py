# Assess landing environment and determine when safe to land

import numpy as np
import random

groundFaceColors = []
for i in range(0, 100):
    n = random.randint(0,3)
    groundFaceColors.append(n)

grid_dim = 5.
drone_pn = -15.
drone_pe = 0.

def assessLand(groundFaceColors, grid_dim, drone_pn, drone_pe):
    safety_map = np.reshape(groundFaceColors, (int(50./grid_dim), int(50./grid_dim)))
    print(safety_map)
    land_zone = []

    east_vals = np.arange(-20, 25, 5)
    north_vals = np.arange(-20, 25, 5)

    for i in range(len(east_vals)):
        if east_vals[i] == drone_pe:
            col1 = i
            col2 = i + 1
    for i in range(len(north_vals)):
        if north_vals[i] == -drone_pn:
            row1 = i
            row2 = i + 1

    land_zone.append(safety_map[row1, col1]) # upper left
    land_zone.append(safety_map[row1, col2]) # upper right
    land_zone.append(safety_map[row2, col1]) # lower left
    land_zone.append(safety_map[row2, col2]) # lower right
    print(land_zone)

    if 3 in land_zone:
        test = str('Unsafe to land, return to van.')
    else:
        land = np.average(land_zone)
        if land <= 1.:
            test = str('Success, package delivered')
        else:
            test = str('Unsafe to land, return to van')
    
    return test

test = assessLand(groundFaceColors, grid_dim, drone_pn, drone_pe)
print(test)