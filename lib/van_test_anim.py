import numpy as np

def van_states(t):

    i = t%10.0
    n0 = -55./2.
    e0 = 55./2.
    pd = 1.0

    phi = np.deg2rad(-90)
    theta = 0
    psi = 0

    if i < 2.5:
        pn = n0 + 55.*((t%2.5)/2.5)
        pe = 55./2.
        psi = np.deg2rad(-90)

    elif i < 5.0:
        pn = 55./2.
        pe = e0 - 55.*((t%2.5)/2.5)
        psi = np.deg2rad(180)
        
    elif i < 7.5:
        pn = -n0 - 55.*((t%2.5)/2.5)
        pe = -e0
        psi = np.deg2rad(90)

    else:
        pn = n0
        pe = -e0 + 55.*((t%2.5)/2.5)
        psi = np.deg2rad(0)

    return [pn,pe,pd,phi,theta,psi]


def van_states2(t):

    #time to drive across each side
    side_time = 10
    
    #side length
    l = 55.0

    #pause time at each corner before turn
    p_time = 0.0

    #total time to drive around
    t_time = (4*(side_time + p_time))

    i = t%t_time
    n0 = -l/2.
    e0 = l/2.
    pd = 0.5

    u = 0
    v = 0
    w = 0

    # Velocity constant while driving
    vel = l/side_time

    phi = np.deg2rad(-90)
    theta = 0
    psi = 0

    #side 1
    if i < side_time:
        pn = n0 + l*((i%side_time)/side_time)
        pe = l/2.
        psi = np.deg2rad(-90)
        u = vel
    
    #side 1 pause
    elif i < (side_time + p_time):
        pn = n0 + l
        pe = l/2
        psi = np.deg2rad(-90)

    #side 2
    elif i < ((2*side_time) + p_time):
        passed_time = (side_time + p_time)
        pn = l/2.
        pe = e0 - l*(i-passed_time)/side_time
        psi = np.deg2rad(180)
        v = -vel

    #side 2 pause
    elif i < (2*(side_time + p_time)):
        pn = l/2.
        pe = -e0
        psi = np.deg2rad(180)

    #side 3
    elif i < ((3*side_time) + (2*p_time)):
        passed_time = 2*(side_time + p_time)
        pn = -n0 - l*(i-passed_time)/side_time
        pe = -e0
        psi = np.deg2rad(90)
        u = -vel

    #side 3 pause
    elif i < (3*(side_time + p_time)):
        pn = n0
        pe = -e0
        psi = np.deg2rad(90)

    #side 4
    elif i < ((4*side_time) + (3*p_time)):
        passed_time = 3*(side_time + p_time)
        pn = n0
        pe = -e0 + l*(i-passed_time)/side_time
        psi = np.deg2rad(0)
        v = vel

    #side 4 pause
    else:
        pn = n0
        pe = e0
        psi = np.deg2rad(0)



    
    return [pn,pe,pd,phi,theta,psi], [u,v,w]

def van_objective(t):
    van_pos, van_vel = van_states2(t)
    if van_vel[0] > 0:
        van_pos[0] += -3
    elif van_vel[0] < 0:
        van_pos[0] += 3
    elif van_vel[1] > 0:
        van_pos[1] += -3
    elif van_vel[1] < 0:
        van_pos[1] += 3
    
    van_pos[2] = 1
    
    return van_pos, van_vel
        

