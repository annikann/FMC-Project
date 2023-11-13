# Objects needed for simulation

import numpy as np
from stl import mesh
import matplotlib.pyplot as plt

import os
import sys
repo = os.getcwd()
libpath = os.path.join(repo, "lib")
stlpath = os.path.join(libpath, "stls")
sys.path.append(stlpath)

# van
van_path = os.path.join(stlpath, 'van.stl')
van_mesh = mesh.Mesh.from_file(van_path)
van_verts = (van_mesh.vectors*np.array([-1, 1, -1]))/30