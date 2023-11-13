# F18 Test Sim (to check general animation file)

from lib import animation
from lib.objects import van_verts
import numpy as np
import matplotlib.pyplot as plt

anim = animation.animation(limits=10, flag=False)
verts = van_verts

pn = 0
pe = 0
pd = 0
phi = 0
theta = 0
psi = 0

anim.update(verts, pn, pe, pd, phi, theta, psi)
plt.show()
