import numpy as np

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