import numpy as np
import lib.quadParameters as P
import desired_states as des

class controller:
    def __init__(self):

        self.kph = P.kph
        self.kdh = P.kdh
        self.kih = P.kih

        self.kpt = P.kpp
        self.kdt = P.kdp
        self.kit = P.kip

        self.kpz = P.kpe
        self.kdz = P.kde
        self.kiz = P.kie

        self.beta = (2.0*P.sigma - P.ts_simulation) / (2.0*P.sigma + P.ts_simulation)

        #integrator and differentiator variables
        self.z_dot = 0.0
        self.z_d1 = 0.0
        self.z_error_d1 = 0.0
        self.z_integrator = 0.0

        self.n_dot = 0.0
        self.n_d1 = 0.0
        self.n_error_d1 = 0.0
        self.n_integrator = 0.0

        self.h_dot = 0.0
        self.h_d1 = 0.0
        self.h_error_d1 = 0.0
        self.h_integrator = 0.0

        self.t_dot = 0.0
        self.t_d1 = 0.0

        self.p_dot = 0.0
        self.p_d1 = 0.0
    

    
    def update_h(self, h_r, y):
        h = -y[2][0]

        f_eq = (P.m) * P.g

        error = h_r - h

        self.h_integrator = self.h_integrator + (P.ts_simulation/2)*(error+self.h_error_d1)

        self.h_dot = (self.beta*self.h_dot) + (1 - self.beta)*((h-self.h_d1)/P.ts_simulation)

        f_squig = (self.kph*error) + (self.kih*self.h_integrator) - (self.kdh*self.h_dot)

        f_unsat = (f_eq + f_squig)
        #f = self.saturate(f_unsat, 2*P.F_max)
        f = f_unsat
        
        if (self.kih != 0.0):
            self.h_integrator = self.h_integrator + P.ts_simulation/self.kih*(f - f_unsat)
        self.h_error_d1 = error
        self.h_d1 = h

        return f
    
    def update_l(self, z_r, z, theta):
        z_error = z_r - z

        self.z_integrator = self.z_integrator + (P.ts_simulation/2)*(z_error+self.z_error_d1)

        self.z_dot = (self.beta*self.z_dot) + (1 - self.beta)*((z-self.z_d1)/P.ts_simulation)

        theta_r_unsat = (self.kpz*(z_error)) - (self.kdz*self.z_dot)

        theta_r = self.saturate(theta_r_unsat, np.deg2rad(15))
        #print(np.deg2rad(theta_r), np.deg2rad(theta))

        if (self.kiz != 0.0):
            self.z_integrator = self.z_integrator + P.ts_simulation/self.kiz*(theta_r - theta_r_unsat)

        t_error = theta_r - theta

        self.t_dot = self.beta * self.t_dot + (1 - self.beta) * ((theta - self.t_d1) / P.ts_simulation)

        tau_unsat = self.kpt*t_error - self.kdt*self.t_dot

        self.z_error_d1 = z_error
        self.z_d1 = z
        self.t_d1 = theta
        return tau_unsat
    
    def update_m(self, z_r, z, theta):
        z_error = z_r - z

        self.n_integrator = self.n_integrator + (P.ts_simulation/2)*(z_error+self.n_error_d1)

        self.n_dot = (self.beta*self.n_dot) + (1 - self.beta)*((z-self.n_d1)/P.ts_simulation)

        theta_r_unsat = (self.kpz*(z_error)) - (self.kdz*self.n_dot)

        theta_r = self.saturate(theta_r_unsat, np.deg2rad(15))
        print(np.deg2rad(theta_r), np.deg2rad(theta))

        if (self.kiz != 0.0):
            self.n_integrator = self.n_integrator + P.ts_simulation/self.kiz*(theta_r - theta_r_unsat)

        t_error = theta_r - theta

        self.p_dot = self.beta * self.p_dot + (1 - self.beta) * ((theta - self.p_d1) / P.ts_simulation)

        tau_unsat = self.kpt*t_error - self.kdt*self.p_dot

        self.n_error_d1 = z_error
        self.n_d1 = z
        self.p_d1 = theta
        return tau_unsat

    
    def update(self, n_r, e_r, h_r, y):
        e_r = -1*e_r
        f = self.update_h(h_r, y)

        n = y[0][0]
        e = -y[1][0]
        phi = y[6][0]
        theta = y[7][0]

        m = self.update_m(n_r, n, theta)

        l = self.update_l(e_r, e, phi)

        return f, l, m
    
    
    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
            #print("Max force reached.")
        return u