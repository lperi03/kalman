##kalman filter in python
#setup with standard A, Q, C, R matrices, can redefine depending on desired state observation vectors
#A --> State transition matrix (describes how states evolve over time)
#Q --> Process Noise covariance matrix (zero mean gaussian)
#C --> Measurement/observation matrix
#R --> Observation/measurement noise covariance matrix (also zero mean gaussian)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy

#Kalman Filter class
class KalmanFilter():
    def __init__(self, A, Q, C, R, state = None):
        self.A = A
        self.Q = Q
        self.C = C
        self.R = R

        ##getting state dimensions from A matrix size
        self.s = A.shape[0]
        self.m = C.shape[0]

        if state is None:
            state = np.zeros(self.s)
        else:
            self.state = state
        self.prev_P = np.zeros((self.s, self.s))
        self.P = np.zeros((self.s, self.s))
        self.steady_state = False

        ##predict method (prediction of next state Xn+1|Xn, using Kalman Equations)

        def predict(self):
            self.prev_P = copy.deepcopy(self.P)
            self.state = self.A @ self.state
            self.P = self.A @ (self.P @ self.A.T) + self.Q

        #update method to update noise covariances
        def update(self, measurement):
            if not self.steady_state:
                #calculaing kalman gain
                self.K = self.P @ (self.C.T @ np.linalg.inv(self.C @ (self.P, self.C.T))+ self.R)
                #updating covariance
                self.P = (np.eye(self.s) - (self.K @ self.C)) @ self.P
                if np.allclose(self.P, self.prev_P):
                    self.steady_state = True

                #calculating innovation and updating state
                innovation = measurement - self.C @ self.state
                self.state = self.state + self.K @ innovation 

        