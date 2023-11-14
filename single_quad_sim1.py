
import sys
sys.path.append('.')# one directory up
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.simulation_parameters as SIM
import parameters.quad_parameters as P
import parameters.quad_pd_gains as pd_gains
from traj.trajectory_gen import Trajectory

from dynamics.quad_dynamics import QuadDynamics
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.df_flatness_traj_following import DF_Traj_Follow

from viewers.quad_animation import quad_animation # for animation 
from viewers.dataPlotter import dataPlotter

show_animation = True
quad_anim = quad_animation(P.state0, size=1, show_animation=show_animation)
att_plot=dataPlotter()    
quad = QuadDynamics(SIM.ts_simulation,P.state0)
myTraj=Trajectory(trajecotry_type=1)
trajFol=DF_Traj_Follow()


roll_pd=PDControlWithRate(kp=pd_gains.roll_kp,kd=pd_gains.roll_kd,ki=pd_gains.roll_ki, Ts=SIM.ts_simulation,limit=pd_gains.tau_phi_max)
pitch_pd=PDControlWithRate(kp=pd_gains.theta_kp,kd=pd_gains.theta_kd,ki=pd_gains.theta_ki, Ts=SIM.ts_simulation,limit=pd_gains.tau_theta_max)
#roll_pd=PDControlWithRate(kp=1,kd=1,limit=2)


# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    phi=quad._state.item(6)
    theta=quad._state.item(7)
    psi=quad._state.item(8)

    p=quad._state.item(9)
    q=quad._state.item(10)
    r=quad._state.item(11)
    [xr, ur]=myTraj.update(sim_time)
    #print('xr=',xr, 'ur=',ur)
    [Ft,phi_c,theta_c,r_c]=trajFol.update(quad._state,xr,ur)
    #phi_c=0.01
    #theta_c=0.01
    #print(Ft,phi_c,theta_c,r_c)
    
    F=P.m*P.g/(np.cos(phi)*np.cos(theta))
    t_phi = roll_pd.update(phi_c,phi,p)
    t_theta = pitch_pd.update(theta_c,theta,q)
    t_psi = 0  # 0.1
    forces_moments = np.array([[float(Ft), t_phi, t_theta, t_psi]]).T

    # -------physical system-------------
    quad.update(forces_moments)  # propagate the MAV dynamics
    
    
    #print(mav._state)

    # update animation 
    quad_anim.update_pose(quad._state.item(0),quad._state.item(1),-quad._state.item(2), quad._state.item(6), quad._state.item(7), quad._state.item(8) )
    att_plot.update(sim_time,phi_c,theta_c,quad._state,t_phi)

    # -------increment time-------------
    sim_time += SIM.ts_simulation