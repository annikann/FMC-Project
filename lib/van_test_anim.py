import numpy as np

def van_states(t):

    i = t%10.0
    n0 = -55./2.
    e0 = 55./2.
    pd = 1.0

    phi = 0
    theta = 0
    psi = 0

    if i < 2.5:
        pn = n0 + 55.*(t/2.5)
        pe = 55./2.5

        
        psi = np.deg2rad(90)
    elif i < 5.0:
        pn = 55./2.
        pe = e0 - 55.*((t%2.5)/2.5)

        psi = np.deg2rad(0)
    elif i < 7.5:
        pn = -n0 - 55.*((t%2.5)/2.5)
        pe = -e0

        psi = np.deg2rad(-90)

    else:
        pn = n0
        pe = -e0 + 55.*((t%2.5)/2.5)

        psi = np.deg2rad(180)

    return [pn,pe,pd,phi,theta,psi]
        
        

