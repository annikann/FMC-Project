import sys
sys.path.append('.')
import numpy as np
from lib.rotations import Euler2Quaternion
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

######################################################################################
                #   Simulation Parameters
######################################################################################
ts_simulation = 0.02  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 400.  # end time for simulation

ts_plotting = 0.1  # refresh rate for plots

ts_video = 0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
pn0 = 1.5  # initial north position
pe0 = 0.  # initial east position
pd0 = -0.75  # initial down position
u0 = 0.  # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0.  # initial velocity along body z-axis
phi0 = 0.  # initial roll angle
theta0 = 0.  # initial pitch angle
psi0 = 0.0  # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate

state0=np.array([[pn0],  
                [pe0], 
                [pd0],
                [u0],   
                [v0], 
                [w0], 
                [phi0], 
                [theta0],
                [psi0],
                [p0],
                [q0],
                [r0]])

#   Quaternion State # not used right now
e = Euler2Quaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)


######################################################################################
                #   Physical Parameters
######################################################################################
# Quadrotr Inertial Parameters
Jx = 0.114700
Jy = 0.057600
Jz = 0.171200
g = 9.806650
m = 1.56
########################################
## Animation Parameters ######
size=0.5
show_animation = True

fig = plt.figure(1)
# for stopping simulation with the esc key.
fig.canvas.mpl_connect('key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

ax = fig.add_subplot(111, projection='3d') # this will be passed around for animation
plt.xlim(-2, 2)
plt.ylim(-2, 2)
ax.set_zlim(0, 4)
plt.xlabel('North')
plt.ylabel('East')
### draw earth
rs = 0.6
us, vs = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
xs = rs*np.cos(us) * np.sin(vs)
ys = rs*np.sin(us) * np.sin(vs)
zs = rs*np.cos(vs)
ax.plot_surface(xs, ys, zs)
#..................................................................