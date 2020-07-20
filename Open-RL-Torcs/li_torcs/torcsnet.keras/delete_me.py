import numpy as np
import matplotlib.pyplot as plt


lqr_data = np.load('lqr_drive_alpine2_3.npz')
lqr_gt_to_middles_m = lqr_data['gt_to_middles_m']
lqr_pred_to_middles_m = lqr_data['pred_to_middles_m']
lqr_gt_angle = lqr_data['gt_angle']
lqr_pred_angle = lqr_data['pred_angle']
# gt_dist = lqr_gt_to_middles_m[::]
# lqr_pred_dist = lqr_pred_to_middles_m[::]
# gt_angle = lqr_gt_angle[::]
# lqr_pred_angle = lqr_pred_angle.squeeze()[::]

mtl_data = np.load('mtl_drive_alpine2_4.npz')
mtl_gt_to_middles_m = mtl_data['gt_to_middles_m']
mtl_pred_to_middles_m = mtl_data['pred_to_middles_m']
mtl_gt_angle = mtl_data['gt_angle']
mtl_pred_angle = mtl_data['pred_angle']

# # gt_dist = mtl_gt_to_middles_m[::]
# # gt_angle = mtl_gt_angle[::]
# mtl_pred_angle = mtl_pred_angle.squeeze()[::]

# err_dist = pred_dist - gt_dist
# err_angle = pred_angle - gt_angle

# err_dist = err_dist.tolist()
# err_dist = [abs(e) for e in err_dist]
# print("mean err_dist = %f, max = %f" % (np.mean(err_dist), np.max(err_dist)))

# err_angle = err_angle.tolist()
# err_angle = [abs(e) for e in err_angle]
# print("mean err_angle = %f, max = %f" % (np.mean(err_angle), np.max(err_angle)))

plt.figure()
plt.subplot(211)
plt.plot(np.zeros(mtl_gt_to_middles_m.shape)[::], '-.', color='g', label = 'Zero line')
plt.plot(lqr_gt_to_middles_m[::], color='#1f77b4', label = 'MTL-LQR')
plt.plot(mtl_gt_to_middles_m[::], color='#ff7f0e', label = 'MTL-RL')
# plt.xlim(-5, 165)
plt.ylim(-1.5, 1.5)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
           ncol=3, mode="expand", borderaxespad=0.)
plt.xlabel('Step')
plt.ylabel('Distance to center (m)')

# Plot angle error
plt.subplot(212)
plt.plot(lqr_gt_angle[::], color='#1f77b4',)
plt.plot(mtl_gt_angle.squeeze()[::], color='#ff7f0e',)
plt.plot(np.zeros(mtl_gt_to_middles_m.shape)[::], '-.', color='g')
# plt.xlim(-5, 165)
plt.ylim(-0.15, 0.15)
plt.xlabel('Step')
plt.ylabel('Yaw angle (rad)')
plt.tight_layout(h_pad=0.05)
# plt.show()
plt.savefig('alpine2_4.png')


lqr_gt_to_middles_m = lqr_gt_to_middles_m.tolist()
lqr_gt_to_middles_m = [abs(e) for e in lqr_gt_to_middles_m]
lqr_gt_angle = lqr_gt_angle.squeeze().tolist()
lqr_gt_angle = [abs(e) for e in lqr_gt_angle]
lqr_mean_dist = np.mean(lqr_gt_to_middles_m)
lqr_mean_angle = np.mean(lqr_gt_angle)


mtl_gt_to_middles_m = mtl_gt_to_middles_m.tolist()
mtl_gt_to_middles_m = [abs(e) for e in mtl_gt_to_middles_m]
mtl_gt_angle = mtl_gt_angle.squeeze().tolist()
mtl_gt_angle = [abs(e) for e in mtl_gt_angle]
mtl_mean_dist = np.mean(mtl_gt_to_middles_m)
mtl_mean_angle = np.mean(mtl_gt_angle)

print('lqr mean dist_err = {}, angle_err = {}'.format(lqr_mean_dist, lqr_mean_angle))
print('mtl mean dist_err = {}, angle_err = {}'.format(mtl_mean_dist, mtl_mean_angle))
