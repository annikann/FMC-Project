# Objects needed for simulation

import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from quality_map import quality
import os
import sys
repo = os.getcwd()

libpath = os.path.join(repo, "lib")
stlpath = os.path.join(libpath, "stls")
sys.path.append(stlpath)

# van
van_path = os.path.join(stlpath, 'assmobile.stl')
drone_path = os.path.join(stlpath, 'X_Shape.stl')
try:
    van_mesh = mesh.Mesh.from_file(van_path)
    drone_mesh = mesh.Mesh.from_file(drone_path)
except:
    van_mesh = mesh.Mesh.from_file(van_path.replace('\\lib\\lib','\\lib'))
    drone_mesh = mesh.Mesh.from_file(drone_path.replace('\\lib\\lib','\\lib'))
    
van_verts = (van_mesh.vectors*np.array([-1, 1, -1]))
drone_verts = (drone_mesh.vectors*np.array([-1, 1, -1]))/30
# Ground
qual, size = quality()
qual_verts = np.zeros([(size**2), 5, 3])
grid_dim = 5 # 1x1 meter grid sqaures

for yi in range(size):
    for xi in range(size):

        #top left corner
        x0 = -size*grid_dim/2
        y0 = size*grid_dim/2

        #point 1:
        x1 = x0 + xi*grid_dim
        y1 = y0 - yi*grid_dim

        #point 2:
        x2 = x1 + grid_dim
        y2 = y1

        #point 3:
        x3 = x2
        y3 = y2 - grid_dim

        #point 4:
        x4 = x1
        y4 = y3

        qual_verts[yi*size + xi,0] = np.array([x1, y1, 0])
        qual_verts[yi*size + xi,1] = np.array([x2, y2, 0])
        qual_verts[yi*size + xi,2] = np.array([x3, y3, 0])
        qual_verts[yi*size + xi,3] = np.array([x4, y4, 0])
        qual_verts[yi*size + xi,4] = np.array([x1, y1, 0])
