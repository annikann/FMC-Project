"""
Class for plotting a quadrotor

Author: Raj # make quad as one object 
"""
import sys
sys.path.append('.')# one directory up
from math import cos, sin
import numpy as np
#import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import parameters.quad_parameters as quad

class quad_animation():
    def __init__(self, state0, size=0.25, show_animation=True):

        self.get_points(size)

        x=state0.item(0)
        y=state0.item(1)
        z=-state0.item(2)

        roll=state0.item(6)
        pitch=state0.item(7)
        yaw=state0.item(8)

        self.show_animation = show_animation

        if self.show_animation:
            #plt.ion()
            self.flag_init = True

            fig = plt.figure(1)
            # for stopping simulation with the esc key.
            fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            self.ax = fig.add_subplot(111, projection='3d')
            self.handle = []

        self.update_pose(x, y, z, roll, pitch, yaw)
    def get_points(self, size):
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p3 = np.array([-size / 2, 0, 0, 1]).T
        self.p2 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.x_data = []
        self.y_data = []
        self.z_data = []

    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
     

        if self.show_animation:
            self.plot()
          

    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = -self.roll
        pitch = -self.pitch
        yaw = self.yaw
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        #plt.cla() # use handle 
        if self.flag_init is True:
            body, =self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0], p1_t[0], p3_t[0], p4_t[0], p2_t[0]],
                        [p1_t[1], p2_t[1], p3_t[1], p4_t[1], p1_t[1], p3_t[1], p4_t[1], p2_t[1]],
                        [p1_t[2], p2_t[2], p3_t[2], p4_t[2], p1_t[2], p3_t[2], p4_t[2], p2_t[2]], 'k-') # rotor
            self.handle.append(body)

            # arm1, =self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
            #             [p1_t[2], p2_t[2]], 'b-')
            # self.handle.append(arm1)
            # arm2, =self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
            #             [p3_t[2], p4_t[2]], 'r-') # arms
            # self.handle.append(arm2)

            traj, =self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')# trajectory
            self.handle.append(traj)

            plt.xlim(-2, 2)
            plt.ylim(-2, 2)
            self.ax.set_zlim(0, 4)
            plt.xlabel('North')
            plt.ylabel('East')
            self.flag_init = False 
            plt.pause(0.001) # can be put in the main file
        else:
            self.handle[0].set_data([p1_t[0], p2_t[0], p3_t[0], p4_t[0], p1_t[0], p3_t[0], p4_t[0], p2_t[0]],
                        [p1_t[1], p2_t[1], p3_t[1], p4_t[1], p1_t[1], p3_t[1], p4_t[1], p2_t[1]])
            self.handle[0].set_3d_properties([p1_t[2], p2_t[2], p3_t[2], p4_t[2],p1_t[2], p3_t[2], p4_t[2], p2_t[2]])

            # self.handle[1].set_data([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]])
            # self.handle[1].set_3d_properties([p1_t[2], p2_t[2]])

            # self.handle[2].set_data([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]])
            # self.handle[2].set_3d_properties([p3_t[2], p4_t[2]])

            self.handle[1].set_data(self.x_data, self.y_data)
            self.handle[1].set_3d_properties(self.z_data)
            print(self.handle)
            plt.pause(0.001)
