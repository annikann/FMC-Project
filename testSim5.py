# Test Sim file (to check general animation file)
# WITH PLOTTING

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states, van_states2, van_objective
from lib.drone_test_anim import drone_states_init
from lib.quadDynamics import quadDynamics
import numpy as np
import matplotlib.pyplot as plt
from lib.PID import controller
import lib.quadParameters as P
from lib.quality_map import LZ_gen
from lib.desired_states import distance, future_van
import lib.envAssess as land
plt.close("all")
plt.ion()

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=True)
drone = quadDynamics()
cont = controller()
anim.QualityMap = qual
q_map = anim.setGroundFaceColors()

anim.van_verts = van_verts
anim.drone_verts = drone_verts

# create subplots
force_plot = plt.figure(1).add_subplot(6, 2, 2)
axpos = force_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 += 0.075; axpos.y1 += 0.075; force_plot.set_position(axpos)
moment_plot = plt.figure(1).add_subplot(6, 2, 4)
axpos = moment_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 += 0.05; axpos.y1 += 0.05; moment_plot.set_position(axpos)
trans_plot = plt.figure(1).add_subplot(6, 2, 6)
axpos = trans_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 += 0.03; axpos.y1 += 0.03; trans_plot.set_position(axpos)
vel_plot = plt.figure(1).add_subplot(6, 2, 8)
axpos = vel_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 += 0.0; axpos.y1 += 0.0; vel_plot.set_position(axpos)
angles_plot = plt.figure(1).add_subplot(6, 2, 10)
axpos = angles_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 -= 0.03; axpos.y1 -= 0.03; angles_plot.set_position(axpos)
angleRs_plot = plt.figure(1).add_subplot(6, 2, 12)
axpos = angleRs_plot.get_position(); axpos.x0 += 0.0; axpos.x1 += 0.0; axpos.y0 -= 0.05; axpos.y1 -= 0.05; angleRs_plot.set_position(axpos)

status = plt.figure(1).add_subplot(6,4,21)
status.spines['bottom'].set_color('white')
status.spines['left'].set_color('white')
status.spines['right'].set_color('white')
status.spines['top'].set_color('white')
status.tick_params(axis='both', which='both', colors='white')
status.text(0., 0.25, 'Drone launched to landing target.', style='italic', bbox={'facecolor': 'yellow', 'alpha': 0.5}, fontsize=15)

sim_times = []
fs = []
ls = []; ms = []; ns = []
pns = []; pes = []; pds = []
n_rs = []; e_rs = []
us = []; vs = []; ws = []
phis = []; thetas = []; psis = []
ps = []; qs = []; rs = []

# States drone can be in
delivering = True
landing = False
returning = False
level = False

on_target = False # Flag for if drone is at target pos

# Timers to assure drone is settled on target
hover_timer = 0.0
land_timer = 0.0
return_timer = 0.0

t = 0
# Initialze drone pos/vels
y = drone.state

# Generate initial LZ
n_r, e_r = LZ_gen()
status.text(0., -0.25,'Delivery Target: ' + str(n_r) + ', ' + str(e_r), fontsize=10)

# n_rs.append(n_r); e_rs.append(e_r)
h_r = 15

plt.pause(2)
while True:
    sim_times.append(t)
    #RUN CHECKS FOR CONTROLLER
        #if at LZ for certain period of time
            #check if safe to land
                #if unsafe gen new LZ & continue
                #if safe then h_r = 1.0
        
        #if at h_r = 0 and within LZ squares
            #drop package (P.m = P.m - package weight)
            #set return to van flag
            #generate initial van location prediction
        #if return to van flag set, continue to generate predition of where van will be based of current velocity and van velocity
        #if drone pos == van pos then we can say fuck this class forever!

    u_r = 0
    v_r = 0
    # Check if safe to land
    if delivering == True:

        # In range of LZ
        if distance(n_r, e_r, h_r, y) < 1.5:
            on_target = True
        # Drifted outside of range, reset timer
        elif distance(n_r, e_r, h_r, y) > 5:
            on_target = False
            level = False
            hover_timer = 0.0
        
        # Update timer
        if on_target == True:
            hover_timer += P.ts_simulation

        if hover_timer > 3.0:
            level = True

        # If sufficiently settled on target
        if hover_timer > 6.0:
            # Assess LZ
            text, safe = land.assessLand(q_map, 5., y[0][0], n_r, y[1][0], e_r)
            print(text)

            # Begin landing procedure
            if safe == True:
                status.clear()
                status.text(0., 0.25, 'Safe to land, deliver package.', style='italic', bbox={'facecolor': 'green', 'alpha': 0.5}, fontsize=15)
                status.text(0., -0.25, 'Delivery Target: ' + str(n_r) + ', ' + str(e_r), fontsize=10)
                h_r = 1.0
                landing = True
                delivering = False
            # Find new LZ
            else:
                status.clear()
                status.text(0., 0.25, 'Not safe to land, find new location.', style='italic', bbox={'facecolor': 'red', 'alpha': 0.5}, fontsize=15)
                n_r, e_r = LZ_gen()
                status.text(0., -0.25,'Delivery Target: ' + str(n_r) + ', ' + str(e_r), fontsize=10)

            
            # Reset timer in case new LZ needed
            hover_timer = 0.0
    
    # Check if landed
    elif landing == True:
        # If near ground in LZ
        if distance(n_r, e_r, h_r, y) <= 5:
            on_target = True
        # Drifted outside of range, reset timer
        elif distance(n_r, e_r, h_r, y) > 5:
            on_target = False
            land_timer = 0.0
        
        # Update timer
        if on_target == True:
            land_timer += P.ts_simulation
        
        # If sufficiently settled on target
        if land_timer > 3.0:
            
            # Update drone mass and controller gains accordingly
            drone.update_m()
            cont.update_gains(False)

            # Package is now delivered            
            print("Package Delivered. Returning to van.")
            status.clear()
            status.text(0., 0.25, 'Package delivered, return to van.', style='italic', bbox={'facecolor': 'green', 'alpha': 0.5}, fontsize=15)

            landing = False
            returning = True
            level = False
    
    # Return to van
    elif returning == True:
        return_timer += P.ts_simulation
        
        # End simulation once drone has returned to van
        n_r, e_r, h_r, u_r, v_r = future_van(t, y)

        if return_timer <= 8.:
            h_r = 5
        elif return_timer <= 13.:
            h_r = -0.8*(return_timer - 8) + 5
        elif return_timer <= 100.:
            n_r, e_r, h_r, u_r, v_r = future_van(t, y)
            # van_pos, van_vel = van_states2(t)
            # u_r = van_vel[0]
            # v_r = van_vel[1]
        
        van_state, van_vel = van_objective(t)
        n_v = van_state[0]
        e_v = van_state[1]
        h_v = van_state[2]
        if distance(n_v, e_v, h_v, y) < 1.75:
            status.clear()
            status.text(0., 0.25, 'Success.', style='italic', bbox={'facecolor': 'green', 'alpha': 0.5}, fontsize=15)
            print("SUCCESS!")
            plt.pause(5)
            break

    # Check if crashed
    if y[2][0] > 0.0:
        print("Drone crashed.")
        status.clear()
        status.text(0., 0.25, 'Drone crashed.', style='italic', bbox={'facecolor': 'red', 'alpha': 0.5}, fontsize=15)
        break
    
    # Update controller forces/moments
    F, l, m, n= cont.update(n_r, e_r, h_r, u_r, v_r, y, level)

    # Propegate dynamics
    y = drone.update(F, l, m, n)

    # Update predetermined van states
    van_state, van_vel = van_states2(t)

    # Update animation
    uav_state = np.array([y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0]])
    anim.update(uav_state, van_state, flag=True)
    anim.ax.plot([e_r - 0.1, e_r + 0.1],[n_r - 0.1, n_r + 0.1], [h_r - 0.1, h_r + 0.1], 'r', linestyle='dashed')  # Plot current controller objective location
    plt.pause(0.05)
    t += P.ts_simulation

    fs.append(F)
    ls.append(l); ms.append(m), ns.append(n)
    pns.append(y[0][0]); pes.append(y[1][0]); pds.append(-1*y[2][0])
    n_rs.append(n_r); e_rs.append(e_r)
    us.append(y[3][0]); vs.append(y[4][0]); ws.append(y[5][0])
    phis.append(y[6][0]); thetas.append(y[7][0]); psis.append(y[8][0])
    ps.append(y[9][0]); qs.append(y[10][0]); rs.append(y[11][0])

    # create plots
    force_plot.clear(); moment_plot.clear()
    trans_plot.clear(); vel_plot.clear(); angles_plot.clear(); angleRs_plot.clear()

    force_plot.plot(sim_times, fs, color='c')
    moment_plot.plot(sim_times, ls, color='c', label='l')
    moment_plot.plot(sim_times, ms, color='m', label='m')
    moment_plot.plot(sim_times, ns, color='pink', label='n')

    trans_plot.plot(sim_times, pns, color='c', label='North')
    trans_plot.plot(sim_times, pes, color='m', label='East')
    trans_plot.plot(sim_times, pds, color='pink', label='Height')
    trans_plot.plot(sim_times, n_rs, color='c', label='North Target', linestyle='dashed')
    trans_plot.plot(sim_times, e_rs, color='m', label='East Target', linestyle='dashed')

    vel_plot.plot(sim_times, us, color='c', label='u')
    vel_plot.plot(sim_times, vs, color='m', label='v')
    vel_plot.plot(sim_times, ws, color='pink', label='w')
    angles_plot.plot(sim_times, np.rad2deg(phis), color='c', label='$\phi$')
    angles_plot.plot(sim_times, np.rad2deg(thetas), color='m', label='$\\theta$')
    angles_plot.plot(sim_times, np.rad2deg(psis), color='pink', label='$\psi$')
    angleRs_plot.plot(sim_times, ps, color='c', label='p')
    angleRs_plot.plot(sim_times, qs, color='m', label='q')
    angleRs_plot.plot(sim_times, rs, color='pink', label='r')

    # Plot titles and labels
    force_plot.grid(); force_plot.set_ylabel('Force (N)')
    moment_plot.legend(loc='upper right', bbox_to_anchor=(1.2,1)); moment_plot.grid(); moment_plot.set_ylabel('Moments (Nm)'); moment_plot.set_ylim(-15, 15)
    trans_plot.legend(loc="upper right", bbox_to_anchor=(1.3,1.05)); trans_plot.grid(); trans_plot.set_ylabel('Position (m)')
    vel_plot.legend(loc="upper right", bbox_to_anchor=(1.2,1)); vel_plot.grid(); vel_plot.set_ylabel('Velocity (m/s)')
    angles_plot.legend(loc="upper right", bbox_to_anchor=(1.2,1)); angles_plot.grid(); angles_plot.set_ylabel('Angle (deg)'); angles_plot.set_ylim(-360,360)
    angleRs_plot.legend(loc="upper right", bbox_to_anchor=(1.2,1)); angleRs_plot.grid(); angleRs_plot.set_ylabel('Angle Rate (deg/s)')
