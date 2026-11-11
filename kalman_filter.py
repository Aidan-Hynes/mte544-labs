

import numpy as np


class kalman_filter:
    
    # Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P
        self.Q=Q
        self.R=R
        self.x=x
        self.dt = dt
        
    # Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # A is the Jacobian of the motion model (state transition matrix)
        self.A = self.jacobian_A()

        # C (often called H) is the Jacobian of the measurement model
        self.C = self.jacobian_H()
        
        self.motion_model()
        
        self.P = self.A @ self.P @ self.A.T + self.Q

    # Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        # compute the Innovation (or Residual) Covariance: S = C*P*C' + R
        S = self.C @ self.P @ self.C.T + self.R

        # compute the Kalman Gain: K = P*C'*inv(S)
        # The Kalman gain dictates how much we trust the measurement vs our prediction
        kalman_gain = self.P @ self.C.T @ np.linalg.inv(S)

        # compute the Measurement Residual (Innovation): Y_bar = z - h(x)
        Y_bar = z - self.measurement_model()

        # update the state estimate x = x + K*Y_bar 
        self.x = self.x + kalman_gain @ Y_bar

        # update the error covariance: P = (I - K*C)*P
        I = np.eye(self.P.shape[0])
        self.P = (I - kalman_gain @ self.C) @ self.P
        
    
    # Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v, linear velocity
            w,# w, angular velocity
            vdot, # ax (linear acceleration in body frame)
            v * w, # ay (cengtripital acceleration in y dir)
        ])
        
    # Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v + vdot*dt,
            vdot,
        ])

    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0, -v * np.sin(th) * dt, 0,np.cos(th) * dt,  0],
            [0, 1, v * np.cos(th) * dt, 0,np.sin(th) * dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0, 0, 0, 0, 1, 0], # v
            [0, 0, 0, 1, 0, 0], # w
            [0, 0, 0, 0, 0, 1], # ax
            [0, 0, 0, v, w, 0], # ay
        ])
        
    # Part 3: return the states here    
    def get_states(self):
        return self.x
