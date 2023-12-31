# Test Sim file (to check general animation file)
# DOES NOT INCLUDE PLOTTING

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states
from lib.drone_test_anim import drone_states
from lib.quadDynamics import quadDynamics
import numpy as np
import matplotlib.pyplot as plt
from lib.PID import controller
import lib.quadParameters as P
plt.close("all")
plt.ion()

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=False)
drone = quadDynamics()
cont = controller()
anim.QualityMap = qual

anim.van_verts = van_verts
anim.drone_verts = drone_verts

y = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])

t = 0
plt.pause(5)
while t < 50:
    F, l, m = cont.update(10, 10, 10, y)
    n = 0.0

    van_state = van_states(t)
    # uav_state = drone_states(t)
    y = drone.update(F, l, m, n)
    uav_state = np.array([y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0]])
    anim.update(uav_state, van_state, flag=False)
    plt.pause(0.05)
    t += P.ts_simulation
