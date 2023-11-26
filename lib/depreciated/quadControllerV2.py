import numpy as np
import lib.quadParameters as P
import desired_states as des

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
        
        self.kds = P.kdp
        self.kps = P.kpp
        
        self.hover = False

    def update_h(self, h_r, state):
        h = -state[2][0]
        h_dot = -state[5][0]
        f_eq = (P.m) * P.g
        f_squig = P.m*(self.kph * (h_r - h)  - self.kdh * h_dot)
        f = (f_eq + f_squig)
        f = self.saturate(f, 100)
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
    
    def update_moments(self, phi_r, theta_r, psi_r, state):
        phi = state[6][0]
        p = state[9][0]
        l = self.kpp * (phi_r - phi) - self.kdp*p

        theta = state[7][0]
        q = state[10][0]
        m = self.kpt*(theta_r - theta) - self.kdt*q

        psi = state[8][0]
        r = state[11][0]
        r_r = 0
        #if (psi_r - psi) > np.deg2rad(180):
            #psi = -1*psi
            #psi_r = -1*psi_r
        if (np.abs(psi-psi_r) > np.deg2rad(2)):
            r_r = np.deg2rad(10.0)
        n = self.kps*(psi_r - psi) + self.kds*(r_r - r)


        return l,m,n
    
    def update_ref(self, n_r, e_r, state):
        n = state[0][0]
        n_dot = state[3][0]
        nddot_r = self.kpn*(n_r - n) - self.kdn*n_dot

        e = state[1][0]
        e_dot = state[4][0]
        eddot_r = self.kpe*(e_r - e) - self.kde*e_dot

        psi_r = des.des_angles(e_r, n_r, state)
        #print("psi_r =", np.rad2deg(psi_r))

        phi_r = (1/P.g) * (eddot_r*np.sin(psi_r) - nddot_r*np.cos(psi_r))
        theta_r = (1/P.g) * (eddot_r*np.cos(psi_r) + nddot_r*np.sin(psi_r))

        return phi_r, theta_r, psi_r

    
    def update(self, n_r, e_r, h_r, state):
        if des.dist(e_r, n_r, state) > 1.5:
            self.hover = False
        if des.dist(e_r, n_r, state) < 1.0:
            self.hover = True
        
        if self.hover == True:
            n_r = state[0][0]
            e_r = state[1][0]
        
        f = self.update_h(h_r, state)
        phi_r, theta_r, psi_r = self.update_ref(n_r, e_r, state)

        l,m,n = self.update_moments(phi_r, theta_r, psi_r, state)
        return f, l, m, n
    
    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
            #print("Max force reached.")
        return u