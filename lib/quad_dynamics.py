
import sys
sys.path.append('.')
import numpy as np

# load message types
from message_types.msg_state import MsgState
import parameters.quad_parameters as quad
from tools.rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation
from math import cos, sin, tan


class QuadDynamics:
    def __init__(self, Ts, state0):
        self.ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 12x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
        self._state=state0
        # self._state = np.array([[quad.north0],  # (0)
        #                        [quad.east0],   # (1)
        #                        [quad.down0],   # (2)
        #                        [quad.u0],    # (3)
        #                        [quad.v0],    # (4)
        #                        [quad.w0],    # (5)
        #                        [quad.phi0],    # (6)
        #                        [quad.theta0],    # (7)
        #                        [quad.psi0],    # (8)
        #                        [quad.p0],    # (9)
        #                        [quad.q0],    # (10)
        #                        [quad.r0]])   # (11)
        self.true_state = MsgState() #figure this out later

    ###################################
    # public functions
    def update(self, forces_moments):
        '''
            Integrate the differential equations defining dynamics. 
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # # normalize the quaternion
        # e0 = self._state.item(6)
        # e1 = self._state.item(7)

        # e2 = self._state.item(8)
        # e3 = self._state.item(9)
        # normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        # self._state[6][0] = self._state.item(6)/normE
        # self._state[7][0] = self._state.item(7)/normE
        # self._state[8][0] = self._state.item(8)/normE
        # self._state[9][0] = self._state.item(9)/normE

        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        #e3 = state.item(9)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)
        #   extract forces/moments
        F = forces_moments.item(0)
        t_phi = forces_moments.item(1)
        t_theta = forces_moments.item(2)
        t_psi= forces_moments.item(3)
        body_vel=np.array([u, v, w]).T
        inertial_vel=Euler2Rotation(phi,theta,psi)@body_vel
        #print(body_vel)
    #print(R_b_w.shape)
        m=quad.m
        g=quad.g
        ThrustVecBody=np.array([0, 0, -F/m]).T
        gravity_vec=np.array([0, 0, g]).T
    #print(ThrustVecBody.shape)
        
    #print(temp1)
      
        #Rgb=rotation_matrix_Gyro2Body(phi, theta)
        
       
       


        # position kinematics
        #pos_dot = 
        north_dot = inertial_vel[0]
        east_dot = inertial_vel[1]
        down_dot = inertial_vel[2]

        # position dynamics # body frame # cross product is missi
        temp1=ThrustVecBody+Euler2Rotation(phi,theta,psi).T@gravity_vec
        u_dot = temp1[0]
        v_dot = temp1[1]
        w_dot = temp1[2]

        # rotational kinematics
        ang_vel=np.array([p, q, r]).T
        Rgb=np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
        [0, cos(phi), -sin(phi)],
        [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
        temp2=Rgb@ang_vel
        phi_dot=temp2[0]
        theta_dot=temp2[1]
        psi_dot=temp2[2]

        # rotatonal dynamics
        Jx=quad.Jx
        Jy=quad.Jy
        Jz=quad.Jz
        p_dot= (Jy - Jz)/Jx * q * r + 1/Jx * t_phi
        q_dot= (Jz - Jx)/Jy * p * r + 1/Jy * t_theta
        r_dot= (Jx - Jy)/Jz * p * q + 1/Jz * t_psi

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]]).T
        #print(x_dot)
        return x_dot

    def _update_true_state(self):
        # update the true state message:
       # phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.phi = self._state.item(6)
        self.true_state.theta = self._state.item(7)
        self.true_state.psi = self._state.item(8)
        self.true_state.p = self._state.item(9)
        self.true_state.q = self._state.item(10)
        self.true_state.r = self._state.item(11)
