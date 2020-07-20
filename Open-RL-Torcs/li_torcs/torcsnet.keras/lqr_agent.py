import os
import tqdm
from control import lqr
import numpy as np


# car1-trb1 parameters
m = 1150  # mass
gcfr = 0.52 # mass repartition between front and rear
lf = 1.2672  # longitudinal distance from C.O.G to front wheel
lr = 1.3728  # longitudinal distance from C.O.G to rear wheel
wheelbase = 2.64  # lf + lr
wheeltrack = 1.64 # lateral distance of left and right wheel center
steer_ratio = 21  # steer ratio
mf = m * gcfr
mr = m * (1 - gcfr)
iz = lf * lf * mf + lr * lr * mr  # rotation inetria
Cf = 80000  # [TBD] cornering stiffness of the front tires
Cr = 80000  # [TBD] cornering stiffness of the rear tires

x = np.zeros((4,1)) # [err_to_middle, d_err_to_middle, err_yaw, d_err_yaw]
A = np.zeros((4,4))
B = np.zeros((4,1))
Q = np.zeros((4,4))
R = np.eye(1)
A[0,1], A[2,3] = 0.5, 0.5
A[1,1] = -1.0*(Cf + Cr)/m  # divide speed vx at running
A[1,2] = (Cf + Cr)/m
A[1,3] = (lf*Cf + lr*Cr)/m  # divide speed vx at running
A[3,1] = -1.0*(lf*Cf - lr*Cr)/iz  # divide speed vx at running
A[3,2] = (lf*Cf - lr*Cr)/iz
A[3,3] = (-1.0*lf*lf*Cf + lr*lr*Cr)/iz  # divide speed vx at running
B[1,0] = Cf/m
B[3,0] = Cf*lf/iz
A *= 2.0
B *= 2.0
freq = 0.05 # bot control frequency
Q[0,0] = 2.0
Q[1,1] = 0.2
Q[2,2] = 2.0
Q[3,3] = 0.1
lambda_ = 0.01
R *= lambda_

class LQRAgent(object):
    def __init__(self):
        self.angle_prev = 0.0
        self.to_middle_prev = 0.0
        self.dt = 0.05

    def predict_lqr_action(self, cnn_laser, speed):
        angle = cnn_laser[0]*3.1416
        angle = angle[0, 0]
        to_middle = cnn_laser[1]*4.0
        angle_rate = (angle - self.angle_prev) / self.dt
        to_middle_rate = (to_middle - self.to_middle_prev) / self.dt
        x = [to_middle, to_middle_rate,
             angle, angle_rate]
        x = np.asarray(x).reshape(-1, 1)

        A[1,1] /= speed + 1e-6
        A[1,3] /= speed + 1e-6
        A[3,1] /= speed + 1e-6
        A[3,3] /= speed + 1e-6
        K, S, E = lqr(A, B, Q, R)
        a = -1.0 * np.dot(K, x)[0,0]
        a = a / steer_ratio
        self.angle_prev = angle
        self.to_middle_prev = to_middle
        A[1,1] = -1.0*(Cf + Cr)/m  # divide speed vx at running
        A[1,3] = (lf*Cf + lr*Cr)/m  # divide speed vx at running
        A[3,1] = -1.0*(lf*Cf - lr*Cr)/iz  # divide speed vx at running
        A[3,3] = (-1.0*lf*lf*Cf + lr*lr*Cr)/iz  # divide speed vx at running
        return a
