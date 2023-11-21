import sys
sys.path.append('.')# one directory up
sys.path.append('.')# one directory up
sys.path.append('C:/Users/tarek/Downloads/flight_sim/flight_sim')
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.simulation_parameters as SIM

#from viewers.quad_animation import quad_animation # for animation 
#from viewers.cube_animation import cube_animation
from viewers.Van import cube_animation
from viewers.cube_animation import drone_animation
#from viewers.dataPlotter import dataPlotter
from tools.signalGenerator import signalGenerator
from tools.sliders import sliders

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
#show_animation = True
#quad_anim = quad_animation(P.state0, size=1, show_animation=show_animation)
state=np.array([[100], [100], [100], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
uav_anim=cube_animation(state, scale=1)
drone_anim=drone_animation(state, scale=1)

my_slider=sliders()
#att_plot=dataPlotter()    
temp = signalGenerator(amplitude=0.5, frequency=0.1)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")

plot0=[]
plot1=[]
plot2=[]
plot3=[]
plot4=[]
plot5=[]
plot6=[]
plot7=[]
plot8=[]
plot9=[]
plot10=[]
plot11=[]

import matplotlib.pyplot as plt
fig, axs = plt.subplots(12)
fig.suptitle('Vertically stacked subplots')


def van_states(t):

    i = t%10.0
    n0 = -55./2.
    e0 = 55./2.
    pd = 1.0

    phi = 0
    theta = 0
    psi = 0

    if i < 2.5:
        pn = n0 + 55.*(t/2.5)
        pe = 55./2.5

        
        psi = np.deg2rad(90)
    elif i < 5.0:
        pn = 55./2.
        pe = e0 - 55.*((t%2.5)/2.5)

        psi = np.deg2rad(0)
    elif i < 7.5:
        pn = -n0 - 55.*((t%2.5)/2.5)
        pe = -e0

        psi = np.deg2rad(-90)

    else:
        pn = n0
        pe = -e0 + 55.*((t%2.5)/2.5)

        psi = np.deg2rad(180)
    state=np.array([[pn], [pe], [pd], [phi], [theta], [psi], [0], [0], [0], [0], [0], [0]])
    return state



# Drone parameters
m = 50.0  # Mass of the drone

# Controller gains
kp = 1.0  # Proportional gain
ki = 0.1  # Integral gain
kd = 0.5  # Derivative gain

# Initial conditions
p_N = np.random.randint(0,150)  # Initial North position
p_E = np.random.randint(0,150)  # Initial East position
p_D = np.random.randint(0,150)  # Initial Down position
v_N = np.random.randint(0,150)  # Initial North velocity
v_E = np.random.randint(0,150)  # Initial East velocity
v_D = np.random.randint(0,150)  # Initial Down velocity
phi_N=50
theta_E=20
psi_D=10
angular_velocity_phi=10
angular_velocity_theta=10
angular_velocity_psi=10

# Setpoint


# Simulation parameters
dt = 0.1  # Time step
total_time = sim_time  # Total simulation time

# Lists to store data for plotting
time_points = []
position_N_points = []
position_E_points = []
position_D_points = []

# PID controller function
def pid_controller(target, current, prev_error, integral):
    error = (target - current)/2 #/10 to take longer time to reach
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral

# PD controller function
def pd_controller(target, current, prev_error):
    error = target - current
    derivative = (error - prev_error) / dt
    output = kp * error + kd * derivative
    return output, error

# Create 3D animation
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')


# Animation update function
def update(frame, ax, position_N_points, position_E_points, position_D_points):
    global p_N, p_E, p_D, v_N, v_E, v_D
    time_points.append(frame * dt)

    # Compute control inputs using PID controller
    control_N, error_N, integral_N = pid_controller(target_position_N, p_N, v_N, 0.3)
    control_E, error_E, integral_E = pid_controller(target_position_E, p_E, v_E, 0.3)
    control_D, error_D, integral_D = pid_controller(target_position_D, p_D, v_D, 0.3)

    # Update accelerations
    acceleration_N = control_N / m
    acceleration_E = control_E / m
    acceleration_D = control_D / m

    # Update velocities and positions
    v_N += acceleration_N * dt
    v_E += acceleration_E * dt
    v_D += acceleration_D * dt
    p_N += v_N * dt
    p_E += v_E * dt
    p_D += v_D * dt

    # Store data for plotting
    position_N_points.append(p_N)
    position_E_points.append(p_E)
    position_D_points.append(p_D)

















while sim_time < SIM.end_time:
    t=sim_time
    state=van_states(sim_time)
    pn=state[0,0]
    pe=state[1,0]
    pd=state[2,0]
    phi=state[3,0]
    theta=state[4,0]
    psi=state[5,0]
    

    
    
    plot0.append(float(pn))
    plot1.append(float(pe))
    plot2.append(float(pd))
    
    plot6.append(float(phi))
    plot7.append(float(theta))
    plot8.append(float(psi))
    
    
    axs[0].plot(t,pn)
    axs[1].plot(t,pe)
    axs[2].plot(t,pd)
    axs[3].plot(t,p_N)
    axs[4].plot(t,p_E)
    axs[5].plot(t,p_D)
    axs[6].plot(t, phi)
    axs[7].plot(t, theta)
    axs[8].plot(t,psi)
    axs[9].plot(t,phi_N)
    axs[10].plot(t,theta_E)
    axs[11].plot(t,psi_D)
    
    uav_anim.update(pn, pe, pd, phi, theta, psi) 
    
    target_position_N = pn# Target North position
    target_position_E = pe   # Target East position
    target_position_D = pd   # Target Down position
    
    target_phi=phi
    target_theta=theta
    target_psi=psi
    
    control_N, error_N, integral_N = pid_controller(target_position_N, p_N, v_N, 0.3)
    control_E, error_E, integral_E = pid_controller(target_position_E, p_E, v_E, 0.3)
    control_D, error_D, integral_D = pid_controller(target_position_D, p_D, v_D, 0.3)
    

    

    control_phi, error_phi, integral_phi = pid_controller(0, phi_N, 0, 0.3)

    
    control_theta, error_theta, integral_theta = pid_controller(0, theta_E, 0, 0.3)

    
    control_psi, error_psi, integral_psi = pid_controller(0, psi_D, 0, 0.3)
    # Update accelerations
    acceleration_N = control_N / m
    acceleration_E = control_E / m
    acceleration_D = control_D / m

    # Update velocities and positions
    v_N += acceleration_N * dt
    v_E += acceleration_E * dt
    v_D += acceleration_D * dt
    p_N += v_N * dt
    p_E += v_E * dt
    p_D += v_D * dt
    
    angular_velocity_phi += control_phi / m
    angular_velocity_theta += control_theta / m
    angular_velocity_psi += control_psi / m

    phi_N += angular_velocity_phi 
    theta_E += angular_velocity_theta 
    psi_D += angular_velocity_psi 


    # Store data for plotting
    plot3.append(p_N)
    plot4.append(p_E)
    plot5.append(p_D)
    
    plot9.append(phi_N)
    plot10.append(theta_E)
    plot11.append(psi_D)
    
    axs[9].plot(t,phi_N)
    axs[10].plot(t,theta_E)
    axs[11].plot(t,psi_D)
    uav_anim.update(pn,pe,pd,phi,theta,psi)
    drone_anim.update(p_N,p_E,p_D,0,0,0)


    # -------increment time-------------
    sim_time += SIM.ts_simulation
    
    axs[0].plot(plot0)
    axs[1].plot(plot1)
    axs[2].plot(plot2)
    axs[3].plot(plot3)
    axs[4].plot(plot4)
    axs[5].plot(plot5)
    axs[6].plot(plot6)
    axs[7].plot(plot7)
    axs[8].plot(plot8)
    axs[9].plot(plot9)
    axs[10].plot(plot10)
    axs[11].plot(plot11)