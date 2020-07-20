import os
import tqdm
from control import lqr
import numpy as np
from environment import Env

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

try_num = 9
track = 'forza'
freq = 0.05 # bot control frequency
noise = 0.005 # noise coef (std)
base = os.path.join('lqr_results', track)
if not os.path.exists(base):
    os.makedirs(base)
Q[0,0] = 1.0
Q[1,1] = 0.2
Q[2,2] = 1.0
Q[3,3] = 0.1
lambda_ = 0.01

R *= lambda_


env = Env()
_ = env.reset()
step = 0
step_limit = 6500
rewards = 0
steers = []
angle_errs = []
dist_errs = []

angle_prev = 0.0
to_middle_prev = 0.0
dt = 0.05

for step in tqdm.tqdm(range(step_limit), ncols = 64):
    x_ = env.read_lqr_x()
    to_middle_m = x_[0] + np.random.randn()*noise*5 # track half_width = 5
    angle = x_[1] + np.random.randn()*noise
    angle_ = x_[1]
    to_middle_rate = (to_middle_m - to_middle_prev) / dt
    angle_rate = (angle - angle_prev) / dt
    to_middle_prev = to_middle_m
    angle_prev = angle
    x = [to_middle_m, to_middle_rate,
         angle, angle_rate]
    x = np.array(x).reshape(-1, 1)

    to_middle = x_[2] # normalized to_middle
    vx = x_[3] # speed_x
    if step > 50:
        A[1,1] /= vx + 1e-6
        A[1,3] /= vx + 1e-6
        A[3,1] /= vx + 1e-6
        A[3,3] /= vx + 1e-6
    r = np.cos(angle_) - np.sin(abs(angle_)) - abs(to_middle)
    rewards += r
    K, S, E = lqr(A, B, Q, R)
    x = np.array(x).reshape(-1,1)
    a = -1.0 * np.dot(K, x)[0,0]
    # convert wheel angle from rad to deg, then normalized by steer ratio
    a = a / steer_ratio
    env.step_lqr(a)
    steers.append(a)
    angle_errs.append(angle_) # unit: rad
    dist_errs.append(x[0]) # unit: m

path_np = os.path.join(base, 'stats_%s.npz'% str(try_num))
np.savez(path_np, steers = steers, dist_errs = dist_errs, angle_errs = angle_errs)
path_f = os.path.join(base, 'results_%s'%str(try_num))
with open(path_f, 'wb') as f:
    f.write('- '*25 + '\n')
    f.write(' '*10 + 'LQR Experiment' + ' '*10 + '\n')
    f.write('- '*25 + '\n')
    f.write('tracks: %s\n' % track)
    f.write('q1 = %f, q2 = %f, q3 = %f, q4 = %f, lambda = %f\n' % (Q[0,0], Q[1,1],
                                                                   Q[2,2], Q[3,3], lambda_))
    f.write('control freq = %f | noise std = %f | scores = %f\n' % (freq, noise, rewards))
    f.write('stats file path: %s\n' % path_np)
print('Result file stored.')
env.end()