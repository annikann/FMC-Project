# Test Sim file (to check general animation file)

import lib.animation as ANIM
from lib.objects import van_verts
from lib.objects import qual_verts, qual
import numpy as np
import matplotlib.pyplot as plt
plt.close("all")
anim = ANIM.animation(limits=25, groundVerts=qual_verts, flag=False)
anim.QualityMap = qual

verts = van_verts

pn = 0
pe = 0
pd = -2
phi = 0
theta = 0
psi = 0
van_state = [pn, pe, pd, phi, theta, psi]
uav_state = van_state

anim.update(verts, uav_state, van_state)
plt.show()
# 