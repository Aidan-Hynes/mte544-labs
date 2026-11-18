

import numpy as np


class kalman_filter:
    
    # Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):

        # state error covariance matrix. Tracks current uncertainty in our state
        # estimate and how errors between states are relatated
        self.P=P

        # process noise covariance matrix. Represents uncertainty in the 
        # kinematic motion model. Higher Q = trust sensors more
        self.Q=Q

        # measurement noise covariance matrix. Represents sensor noise from the 
        # odometry and imu. Higher R = trust model more
        self.R=R

        # state vector
        self.x=x

        # discrete time step
        self.dt = dt
        
    # Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # A is the Jacobian of the motion model (state transition matrix)
        self.A = self.jacobian_A()

        # C (often called H) is the Jacobian of the measurement model
        self.C = self.jacobian_H()

        # project the state estimate forward
        self.motion_model()

        # Propagate the state error covariance forward: P = A * P * A^T + Q
        self.P = self.A @ self.P @ self.A.T + self.Q

    # Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        # compute the Innovation (or Residual) Covariance: S = C*P*C' + R
        # this combines the projected state uncertainty with measurement noise R
        S = self.C @ self.P @ self.C.T + self.R

        # compute the Kalman Gain: K = P*C'*inv(S)
        # The Kalman gain dictates how much we trust the measurement vs our prediction
        kalman_gain = self.P @ self.C.T @ np.linalg.inv(S)

        # compute the Measurement Residual (Innovation): Y_bar = z - h(x)
        # measures discrepancy between actual sensor readings z and what
        # filter expected (h(x))
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
            v,# v, linear velocity from odom
            w,# w, angular velocity from odom
            vdot, # ax (linear acceleration in body frame) from imu
            v * w, # ay (centripital acceleration in y dir) from imu
        ])
        
    # Part 3: Impelment the motion model (state-transition matrice)
    # this function propogates the robot's pose and velocities forward by dt
    # assuming constant linear and angular velocity
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt, # X position integrated along heading
            y + v * np.sin(th) * dt, # Y position ingtegrated along heading
            th + w * dt, # Heading integrated from angular velocity
            w, # angular velocity held constant over dt
            v + vdot*dt, # linear velocity integrated from acceleration
            vdot,  # linear acceleration held constant
        ])


    # computes the Jacobian matrix of the motion model
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0, -v * np.sin(th) * dt, 0,np.cos(th) * dt,  0], # dx_new/dx_j
            [0, 1, v * np.cos(th) * dt, 0,np.sin(th) * dt,  0], # dy_new/dx_j
            [0, 0,                1, dt,           0,  0], # dth_new/dx_j
            [0, 0,                0, 1,            0,  0], # dw_new/dx_j
            [0, 0,                0, 0,            1,  dt], # dv_new/dx_j
            [0, 0,                0, 0,            0,  1 ] # dvdot_new/dx_j
        ])
    
    
    # Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0, 0, 0, 0, 1, 0], # dv/dx_j
            [0, 0, 0, 1, 0, 0], # dw/dx_j
            [0, 0, 0, 0, 0, 1], # dax/dx_j
            [0, 0, 0, v, w, 0], # day/dx_j: d(v*w)/dw = v, d(v*w)/dv = w
        ])
        
    # Part 3: return the states here    
    def get_states(self):
        return self.x
