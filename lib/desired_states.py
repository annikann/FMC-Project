import numpy as np

def des_angles(des_x, des_y, states):
    drone_x = states.item(1)
    drone_y = states.item(0)

    des_phi = np.arctan2((des_x - drone_x), (des_y - drone_y))

    if des_phi < 0:
        des_phi = (2*np.pi) + des_phi

    return des_phi

def dist(des_x, des_y, states):
    drone_x = states.item(1)
    drone_y = states.item(0)
    dist = np.sqrt((des_x - drone_x)**2 + (des_y - drone_y)**2)
    return dist