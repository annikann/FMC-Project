# Test Sim file (to check general animation file)
# DOES NOT INCLUDE PLOTTING

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states, van_states2
from lib.drone_test_anim import drone_states_init
from lib.quadDynamics import quadDynamics
import numpy as np
import matplotlib.pyplot as plt
from lib.PID import controller
import lib.quadParameters as P
from lib.quality_map import LZ_gen
from lib.desired_states import dist
import lib.envAssess as land
plt.close("all")
plt.ion()

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=False)
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

on_target = False # Flag for if drone is at target pos

# Timers to assure drone is settled on target
hover_timer = 0.0
land_timer = 0.0

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

    
    # Check if safe to land
    if delivering == True:
        # In range of LZ
        if dist(e_r, n_r, y) < 1.5:
            on_target = True
        # Drifted outside of range, reset timer
        elif dist(e_r, n_r, y) > 5:
            on_target = False
            hover_timer = 0.0
        
        # Update timer
        if on_target == True:
            hover_timer += P.ts_simulation

        # If sufficiently settled on target
        if hover_timer > 3.0:
            # Assess LZ
            text, safe = land.assessLand(q_map, 5., y[0][0], n_r, y[1][0], e_r)
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
        if dist(e_r, n_r, y) < 1.5:
            on_target = True
        # Drifted outside of range, reset timer
        elif dist(e_r, n_r, y) > 5:
            on_target = False
            land_timer = 0.0
        
        # Update timer
        if on_target == True:
            land_timer += P.ts_simulation
        
        # If sufficiently settled on target
        if land_timer > 3.0:
#######################################################################
            # NEED TO UPDATE DRONE PARAMS HERE
#######################################################################

            # Package is now delivered            
            print("Package Delivered. Returning to van.")
            landing = False
            returning = True
    
    # Return to van
    elif returning == True:
#######################################################################
            # NEED TO PREDICT FUTURE VAN LOCATION HERE
#######################################################################
        n_r = van_state[0]
        e_r = van_state[1]

        # End simulation once drone has returned to van
        if dist(e_r, n_r, y) < 0.1:
            print("SUCCESS!")
            break
    

    # Update controller forces/moments
    F, l, m, n= cont.update(n_r, e_r, h_r, y)

    # Propegate dynamics
    y = drone.update(F, l, m, n)

    # Update predetermined van states
    van_state, van_vel= van_states2(t)

    # Update animation
    uav_state = np.array([y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0]])
    anim.update(uav_state, van_state, flag=True)
    anim.ax.plot([e_r-0.1,e_r+0.1],[n_r-0.1,n_r+0.1], [h_r-0.1,h_r+0.1], 'r')  # Plot current controller objective location
    plt.pause(0.05)
    t += P.ts_simulation
