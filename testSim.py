# Test Sim file (to check general animation file)

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states
from lib.drone_test_anim import drone_states
from lib.quadDynamics import quadDynamics
import numpy as np
import matplotlib.pyplot as plt
plt.close("all")

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=False)
drone = quadDynamics()
anim.QualityMap = qual

anim.van_verts = van_verts
anim.drone_verts = drone_verts

t = 0
# plt.pause(5)
while t < 10:
        
    if t < 4:
        F = 12.
        l = 0.
        m = 0.
        n = 0.

    van_state = van_states(t)
    # uav_state = drone_states(t)
    y = drone.update(F, l, m, n)
    uav_state = np.array([y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0]])
    print(uav_state)
    anim.update(uav_state, van_state, flag=False)
    print(t)
    plt.pause(0.001)
    t += 0.05
    break
    
# 