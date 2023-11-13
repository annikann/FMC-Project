# Test Sim file (to check general animation file)

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states
from lib.drone_test_anim import drone_states
import numpy as np
import matplotlib.pyplot as plt
plt.close("all")

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=False)
anim.QualityMap = qual

anim.van_verts = van_verts
anim.drone_verts = drone_verts

t = 7
while t < 10:
    van_state = van_states(t)
    uav_state = drone_states(t)
    anim.update(uav_state, van_state)
    print(t)
    plt.pause(0.001)
    t += 0.1
# 