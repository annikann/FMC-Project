import numpy as np

def drone_states(t):
    if t <= 3:
        pn = 0
        pe = 0
        pd = -2*t

        phi = 0
        theta = 0
        psi = (t-3)
    if t > 3:
        pd = -6
        pn = 10*np.sin(t - 3)
        pe = 10*np.sin(t-3)

        vn = np.cos(t-3)
        ve = -np.sin(t-3)

        phi = vn*np.deg2rad(30)
        theta = ve*np.deg2rad(30)
        psi = 0

    return [pn,pe,pd,phi,theta,psi]