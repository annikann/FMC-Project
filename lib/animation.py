"""
FMC Final Project
Team Nozama

Animation class for 3D simulation of docking drone.
"""

import sys
import os
repo = os.getcwd() 
libpath = os.path.join(repo, "lib")
sys.path.append(libpath)
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits.mplot3d import Axes3D
from rotations import Euler2Rotation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class animation():

    def __init__(self, limits, flag=False):

        self.flag_init = True
        self.fig = plt.figure(1)
        if flag == True:
            self.ax = self.fig.add_subplot(1, 3, 2, projection="3d") 
        else:
            self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-limits,limits])
        self.ax.set_ylim([-limits,limits])
        self.ax.set_zlim([-limits,limits])
        self.lim = limits
        self.ax.set_title('3D Animation')
        self.ax.set_xlabel('East(m)')
        self.ax.set_ylabel('North(m)')
        self.ax.set_zlabel('Height(m)')

    def rotate_translate(self, vertices, pn, pe, pd, phi, theta, psi):

        pos_ned = np.array([pn, pe, pd])

        # create m by n copies of pos_ned and used for translation
        ned_rep = np.tile(pos_ned.copy(), (np.shape(vertices)[0], 1))

        R = Euler2Rotation(phi,theta,psi)
        vertRot = np.matmul(R, vertices.T).T
        vertTrans = vertRot + ned_rep

        # rotate for plotting north = y, east = x, h = -z
        R_plot = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, -1]])
        
        newVerts = np.matmul(R_plot, vertTrans.T).T

        return(newVerts)

    def update(self, vertices, pn, pe, pd, phi, theta, psi):
        # draw object
        self.draw(vertices, pn, pe, pd, phi, theta, psi)

        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False


    def draw(self, vertices, pn, pe, pd, phi, theta, psi):

        vertices = np.reshape(vertices, (-1, 3)) # reshapes to (N, 3) based on given vertices
        obj_verts = self.rotate_translate(vertices, pn, pe, pd, phi, theta, psi)
        faces = np.reshape(obj_verts, (-1, 3, 3)) 

        if self.flag_init is True:
            poly = Poly3DCollection(faces, alpha=.6)
            self.object = self.ax.add_collection3d(poly)
            self.ax.set_xlim([pe-self.lim, pe+self.lim])
            self.ax.set_ylim([pn-self.lim, pn+self.lim])
            self.ax.set_zlim([-pd-self.lim, -pd+self.lim])
            plt.pause(0.01)
        else:
            self.object.set_verts(faces)
            self.ax.set_xlim([pe-self.lim, pe+self.lim])
            self.ax.set_ylim([pn-self.lim, pn+self.lim])
            self.ax.set_zlim([-pd-self.lim, -pd+self.lim])
            plt.pause(0.01)