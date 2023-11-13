import numpy as np

def drone_states(t):
    if t <= 5:
        pn = 0
        pe = 0
        pd = -t

        phi = 0
        theta = 0
        psi = (t-5)
    if t > 5:
        pd = -5
        pn = 10*np.sin(t - 5)
        pe = 10*np.cos(t-5)

        vn = np.cos(t-5)
        ve = -np.sin(t-5)

        phi = vn*np.deg2rad(30)
        theta = ve*np.deg2rad(30)
        psi = 0

    return [pn,pe,pd,phi,theta,psi]