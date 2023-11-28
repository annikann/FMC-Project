import numpy as np
from van_test_anim import van_states2

def des_angles(des_x, des_y, states):
    drone_x = states.item(1)
    drone_y = states.item(0)

    des_psi = np.arctan2((des_x - drone_x), (des_y - drone_y))

    if des_psi < 0:
        des_psi = (2*np.pi) + des_psi

    #if des_x < 0:
        #des_psi += np.deg2rad(90)
    psi = states[8][0]

    if (psi - des_psi) > np.deg2rad(180):
        des_psi += np.deg2rad(360)

    return des_psi

def dist(des_x, des_y, states):
    drone_x = states.item(1)
    drone_y = states.item(0)
    dist = np.sqrt((des_x - drone_x)**2 + (des_y - drone_y)**2)
    return dist

def distance(des_n, des_e, des_h, states):
    n = states.item(0)
    e = states.item(1)
    h = -states.item(2)

    dist = np.sqrt((des_n - n)**2 + (des_e - e)**2 + (des_h - h)**2)
    return dist

def future_van(t, states):
    van_pos, van_vel = van_states2(t)
    van_n = van_pos[0]
    van_e = van_pos[1]
    d = dist(van_e, van_n, states)
    #if d <= 10:
        #t = t_pred
    #else: t += d/10 + 0.5
    t += d/10 + 0.5

    future_pos, future_vel = van_states2(t)
    n_r = future_pos[0]
    e_r = future_pos[1]
    h_r = 1.0
    u_r = future_vel[0]/2.
    v_r = future_vel[1]/2.

    return n_r, e_r, h_r, u_r, v_r