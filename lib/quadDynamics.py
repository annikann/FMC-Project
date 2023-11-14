
import sys
import numpy as np
import lib.quadParameters as P
from lib.rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation
from math import cos, sin, tan

class quadDynamics:
    def __init__(self):
        self.Ts = P.ts_simulation
        # set initial states based on parameter file
        self.state = P.state0
        self.m = P.m
        self.g = P.g
        self.Jx = P.Jx
        self.Jy = P.Jy
        self.Jz = P.Jz

    def update(self, fx, fy, fz, l, m, n):
        # saturate the input force
        self.rk4_step(fx, fy, fz, l, m, n)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, forces_moments):

        # extract the states
        pn = state[0][0]
        pe = state[1][0]
        pd = state[2][0]
        u = state[3][0]
        v = state[4][0]
        w = state[5][0]
        phi = state[6][0]
        theta = state[7][0]
        psi = state[8][0]
        p = state[9][0]
        q = state[10][0]
        r = state[11][0]

        #   extract forces/moments
        F = forces_moments.item(0)
        t_phi = forces_moments.item(1)
        t_theta = forces_moments.item(2)
        t_psi = forces_moments.item(3)

        ThrustVecBody = np.array([0, 0, -F/self.m]).T
        gravity_vec = np.array([0, 0, self.g]).T

        # position kinematics
        body_vel = np.array([u, v, w]).T
        inertial_vel = Euler2Rotation(phi, theta, psi)@body_vel
        pndot = inertial_vel[0]
        pedot = inertial_vel[1]
        pddot = inertial_vel[2]

        # position dynamics # body frame # cross product is missi
        temp1 = ThrustVecBody + Euler2Rotation(phi, theta, psi).T@gravity_vec
        udot = temp1[0]
        vdot = temp1[1]
        wdot = temp1[2]

        # rotational kinematics
        ang_vel = np.array([p, q, r]).T
        Rgb = np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                        [0, cos(phi), -sin(phi)],
                        [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
        temp2 = Rgb@ang_vel
        phidot = temp2[0]
        thetadot = temp2[1]
        psidot = temp2[2]

        # rotatonal dynamics
        pdot = (self.Jy - self.Jz)/self.Jx * q * r + 1/self.Jx * t_phi
        qdot = (self.Jz - self.Jx)/self.Jy * p * r + 1/self.Jy * t_theta
        rdot = (self.Jx - self.Jy)/self.Jz * p * q + 1/self.Jz * t_psi

        # collect the derivative of the states
        x_dot = np.array([[pndot, pedot, pddot, udot, vdot, wdot, phidot, thetadot, psidot, pdot, qdot, rdot]]).T
        #print(x_dot)
        return x_dot
    
    def h(self):
        pn = self.state[0][0]
        pe = self.state[1][0]
        pd = self.state[2][0]
        u = self.state[3][0]
        v = self.state[4][0]
        w = self.state[5][0]
        phi = self.state[6][0]
        theta = self.state[7][0]
        psi = self.state[8][0]
        p = self.state[9][0]
        q = self.state[10][0]
        r = self.state[11][0]
        y = np.array([[pn], [pe], [pd], [u], [v], [w], [phi], [theta], [psi], [p], [q], [r]], dtype=float)
        return y

    def rk4_step(self, fx, fy, fz, l, m, n):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.f(self.state, fx, fy, fz, l, m, n)
        k2 = self.f(self.state + self.Ts/2.*k1, fx, fy, fz, l, m, n)
        k3 = self.f(self.state + self.Ts/2.*k2, fx, fy, fz, l, m, n)
        k4 = self.f(self.state + self.Ts*k3, fx, fy, fz, l, m, n)
        self.state += self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
