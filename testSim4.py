# Test Sim file (to check general animation file)
# DOES NOT INCLUDE PLOTTING

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
h_r = 15

#plt.pause(5)
while True:

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
            text,safe = land.assessLand(q_map, 5., y[0][0], n_r, y[1][0], e_r)
            print(text)

            # Begin landing procedure
            if safe == True:
                h_r = 1.0
                landing = True
                delivering = False
            # Find new LZ
            else:
                n_r, e_r = LZ_gen()
            
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
            landing = False
            returning = True
            level = False
    
    # Return to van
    elif returning == True:
        return_timer += P.ts_simulation
        
        # Predict future van location based off distance
        n_r, e_r, h_r, u_r, v_r = future_van(t, y)

        # End simulation once drone has returned to van
        if return_timer < 15.:  # gives the drone some time to match van before dropping
            van_state, van_vel = van_objective(t)
            n_v = van_state[0]
            e_v = van_state[1]
            h_r = 8
        
        elif return_timer < 100.:
            van_state, van_vel = van_objective(t)
            n_v = van_state[0]
            e_v = van_state[1]
            h_v = van_state[2]
            if distance(n_v, e_v, h_v, y) < 2:
                print("SUCCESS!")
                break

    # Check if crashed
    if y[2][0] > 0.0:
        print("Drone crashed.")
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
    anim.ax.plot([e_r-0.1,e_r+0.1],[n_r-0.1,n_r+0.1], [h_r-0.1,h_r+0.1], 'r')  # Plot current controller objective location
    plt.pause(0.05)
    t += P.ts_simulation
