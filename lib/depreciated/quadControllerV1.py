import numpy as np
import lib.quadParameters as P


class controller:
    def __init__(self):
        self.kpt = P.kpt
        self.kdt = P.kdt

        self.kpp = P.kpp
        self.kdp = P.kdp

        self.kpn = P.kpn
        self.kdn = P.kdn

        self.kpe = P.kpe
        self.kde = P.kde

        self.kph = P.kph
        self.kdh = P.kdh

    def update_h(self, h_r, state):
        h = -state[2][0]
        h_dot = -state[5][0]
        f_eq = (P.m) * P.g
        f_squig = self.kph * (h_r - h)  - self.kdh * h_dot
        f = (f_eq + f_squig)
        f = self.saturate(f, 50)
        return f
    
    def update_l(self, e_r, state):
        e = state[1][0]
        theta = state[7][0]
        e_dot = state[4][0]
        theta_dot = state[10][0]

        theta_r = (self.kpe*(e_r - e)) - (self.kde*e_dot)

        l = (self.kpt*(theta_r - theta)) - (self.kdt*theta_dot)

        l = self.saturate(l, 5)

        return l
    
    def update_m(self, n_r, state):
        n = state[0][0]
        phi = state[6][0]
        n_dot = state[3][0]
        phi_dot = state[9][0]

        phi_r = (self.kpn*(n_r - n)) - (self.kdn*n_dot)

        m = (self.kpp*(phi_r - phi)) - (self.kdp*phi_dot)

        m = self.saturate(m, 5)

        return m
    
    def update(self, n_r, e_r, h_r, state):
        f = self.update_h(h_r, state)
        l = self.update_l(e_r, state)
        m = self.update_l(n_r, state)
        return f, l, m
    
    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
            print("Max force reached.")
        return u