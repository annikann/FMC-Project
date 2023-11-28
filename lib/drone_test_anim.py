import numpy as np

def drone_states_init(van_states, van_vel):
    phi = 0
    theta = 0
    psi  = 0
    p = 0
    q = 0
    r = 0

    pn = van_states[0]
    pe = van_states[1]
    pd = van_states[2] - 1.0
    
    u = van_vel[0]
    v = van_vel[1]
    w = van_vel[2]

    return np.array([[pn],[pe],[pd],[u],[v],[w],[phi],[theta],[psi],[p],[q],[r]])