# Test Vertices
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import os
import sys
repo = os.getcwd()
libpath = os.path.join(repo, "lib")
stlpath = os.path.join(libpath, "stls")
sys.path.append(stlpath)

# F-18
f18_path = os.path.join(stlpath, 'f18.stl')
f18_mesh = mesh.Mesh.from_file(f18_path)
f18_verts = f18_mesh.vectors*np.array([-1, 1, -1])