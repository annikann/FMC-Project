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

    def __init__(self, limits, groundVerts, flag=False):
        self.flag_init = True
        self.fig = plt.figure(1)
        if flag == True:
            self.ax = self.fig.add_subplot(1, 3, 2, projection="3d") 
        else:
            self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-limits,limits])
        self.ax.set_ylim([-limits,limits])
        self.ax.set_zlim([-0.1,limits])
        self.lim = limits
        self.ax.set_title('3D Animation')
        self.ax.set_xlabel('East(m)')
        self.ax.set_ylabel('North(m)')
        self.ax.set_zlabel('Height(m)')
    
        self.drone_verts = None
        self.van_verts = None
        self.ground_faces = np.reshape(groundVerts, (-1, 5, 3)) #self.genGroundFaces(groundVerts)
        self.QualityMap = None
       
    def setGroundFaceColors(self):
        landQualMap = self.QualityMap
        nRows, nCols = np.shape(landQualMap)
        color_choices = ['tab:red','tab:orange','y','tab:green']
        self.groundFaceColors = []
        for r in range(nRows):
            for c in range(nCols):
                self.groundFaceColors.append(color_choices[int(landQualMap[c,r])-1])
        
    
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

    def update(self, UAV_state, VAN_state):
        # draw object
        # self.draw_ground()
        self.draw_van(VAN_state)
        self.draw_drone(UAV_state)
        

        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False


    def draw_van(self, state_array): # Van and UAV
        [pn, pe, pd, phi, theta, psi] = state_array
        vertices = np.reshape(self.van_verts, (-1, 3)) # reshapes to (N, 3) based on given vertices
        obj_verts = self.rotate_translate(vertices, pn, pe, pd, phi, theta, psi)
        faces = np.reshape(obj_verts, (-1, 3, 3)) 

        if self.flag_init is True:
            poly = Poly3DCollection(faces, alpha=1)
            self.van = self.ax.add_collection3d(poly)
            # self.ax.set_xlim([pe-self.lim, pe+self.lim])
            # self.ax.set_ylim([pn-self.lim, pn+self.lim])
        else:
            self.van.set_verts(faces)
            # self.ax.set_xlim([pe-self.lim, pe+self.lim])
            # self.ax.set_ylim([pn-self.lim, pn+self.lim])

            
    def draw_drone(self, state_array):
        [pn, pe, pd, phi, theta, psi] = state_array
        vertices = np.reshape(self.drone_verts, (-1, 3)) # reshapes to (N, 3) based on given vertices
        obj_verts = self.rotate_translate(vertices, pn, pe, pd, phi, theta, psi)
        faces = np.reshape(obj_verts, (-1, 3, 3)) 

        if self.flag_init is True:
            poly = Poly3DCollection(faces, alpha=1)
            self.drone = self.ax.add_collection3d(poly)
            # self.ax.set_xlim([pe-self.lim, pe+self.lim])
            # self.ax.set_ylim([pn-self.lim, pn+self.lim])
        else:
            self.drone.set_verts(faces)
            # self.ax.set_xlim([pe-self.lim, pe+self.lim])
            # self.ax.set_ylim([pn-self.lim, pn+self.lim])
            
        
    def draw_ground(self):
        # print('ground')
        if self.flag_init is True:
            self.setGroundFaceColors()
            poly = Poly3DCollection(self.ground_faces, alpha=.6, facecolors=self.groundFaceColors)
            self.ground = self.ax.add_collection3d(poly)

        else:
            self.object.set_verts(self.ground_faces)
    
    def genGroundFaces(self, verts):
        # np.zeros([(size**2), 5, 3])
        faces = []
        numF, numV, xyz = np.shape(verts)
        
        for f in range(numF):
            f_x = []
            f_y = []
            f_z = []
            
            for v in range(numV):
                f_x.append(verts[f,v,0])
                f_y.append(verts[f,v,1])
                f_z.append(verts[f,v,2])
            
                face = np.vstack((np.array(f_x),np.array(f_y),np.array(f_z)))
                faces.append[face]
            
        tran_faces = np.transpose(np.array(faces), (2, 1, 0))
        return tran_faces
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        