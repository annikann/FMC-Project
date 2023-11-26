# Test Sim file (to check general animation file)
# INCLUDES PLOTTING

import lib.animation as ANIM
from lib.objects import van_verts, drone_verts
from lib.objects import qual_verts, qual
from lib.van_test_anim import van_states
from lib.drone_test_anim import drone_states
from lib.quadDynamics import quadDynamics
from lib.quadController2 import controller
import lib.envAssess as land
import lib.quadParameters as P
import numpy as np
import matplotlib.pyplot as plt
plt.close("all")
plt.ion()

anim = ANIM.animation(limits=30, groundVerts=qual_verts, flag=False)
drone = quadDynamics()
cont = controller()
anim.QualityMap = qual

anim.van_verts = van_verts
anim.drone_verts = drone_verts

# create subplots
force_plot = plt.figure(2).add_subplot(6, 1, 1)
# axpos = force_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 -= 0.05; axpos.y1 -= 0.05; force_plot.set_position(axpos)
moment_plot = plt.figure(2).add_subplot(6, 1, 2)
# axpos = moment_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 -= 0.1; axpos.y1 -= 0.1; moment_plot.set_position(axpos)
trans_plot = plt.figure(2).add_subplot(6, 1, 3)
# axpos = trans_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 += 0.03; trans_plot.set_position(axpos)
vel_plot = plt.figure(2).add_subplot(6, 1, 4)
# axpos = vel_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 += 0.03; vel_plot.set_position(axpos)
angles_plot = plt.figure(2).add_subplot(6, 1, 5)
# axpos = angles_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 += 0.03; angles_plot.set_position(axpos)
angleRs_plot = plt.figure(2).add_subplot(6, 1, 6)
# axpos = angleRs_plot.get_position(); axpos.x0 += 0.02; axpos.x1 += 0.02; axpos.y0 += 0.03; angleRs_plot.set_position(axpos)

sim_times = []
fs = []
ls = []; ms = []; ns = []
pns = []; pes = []; pds = []
us = []; vs = []; ws = []
phis = []; thetas = []; psis = []
ps = []; qs = []; rs = []

y = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])

t = 0
# plt.pause(5)
while t < 30:
    sim_times.append(t)

    n_r = 5
    e_r = 5
    h_r = 10
    F, l, m, n = cont.update(n_r, e_r, h_r, y)
    n = 0.0
    print(F, l, m, n)

    van_state = van_states(t)
    # uav_state = drone_states(t)
    y = drone.update(F, l, m, n)
    uav_state = np.array([y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0]])
    anim.update(uav_state, van_state, flag=False)
    safety = anim.setGroundFaceColors()
    test = land.assessLand(safety, 5., y[0][0], n_r, y[1][0], e_r)
    plt.pause(0.05)
    t += P.ts_simulation

    fs.append(F)
    ls.append(l); ms.append(m), ns.append(n)
    pns.append(y[0][0]); pes.append(y[1][0]); pds.append(-1*y[2][0])
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
    moment_plot.legend(loc='upper left'); moment_plot.grid(); moment_plot.set_ylabel('Moments (Nm)'); moment_plot.set_ylim(-50, 50)
    trans_plot.legend(loc="upper left"); trans_plot.grid(); trans_plot.set_ylabel('Position (m)')
    vel_plot.legend(loc="upper left"); vel_plot.grid(); vel_plot.set_ylabel('Velocity (m/s)')
    angles_plot.legend(loc="upper left"); angles_plot.grid(); angles_plot.set_ylabel('Angle (deg)'); angles_plot.set_ylim(-360,360)
    angleRs_plot.legend(loc="upper left"); angleRs_plot.grid(); angleRs_plot.set_ylabel('Angle Rate (deg/s)')

    print(test) # right now the land assessment works when giving it the target coords
