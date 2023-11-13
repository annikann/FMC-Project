# F18 Test Sim (to check general animation file)

from lib import animation
from lib.f18Test import f18_verts
import numpy as np
import matplotlib.pyplot as plt

anim = animation.animation(limits=20, flag=False)
verts = f18_verts

pn = 0
pe = 0
pd = 0
phi = 0
theta = 0
psi = 0

anim.update(verts, pn, pe, pd, phi, theta, psi)
plt.waitforbuttonpress()

