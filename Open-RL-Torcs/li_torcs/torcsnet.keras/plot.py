import numpy as np
import matplotlib.pyplot as plt


data = np.load('ai_drive_stats_mtl_unkonwn_trk.npz')
gt_angle = data['gt_angle']
pred_angle = data['pred_angle'].squeeze()
gt_to_middle_m = data['gt_to_middles_m']
pred_to_middle_m = data['pred_to_middles_m'].squeeze()

plt.subplot(211)
plt.plot(gt_to_middle_m[::20], color='#1f77b4', label = 'Groundtruth')
plt.plot(pred_to_middle_m[::20], '--', color='#2ca02c', label = 'MTL')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
           ncol=3, mode="expand", borderaxespad=0.)
plt.xlim(-5, 125)
plt.ylim(-1.5, 1.5)
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')

plt.subplot(212)
plt.plot(gt_angle[::20], color='#1f77b4', label = 'Groundtruth')
plt.plot(pred_angle[::20], '--', color='#2ca02c', label = ' MTL')
plt.xlim(-5, 125)
plt.ylim(-0.15, 0.15)
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (rad)')
plt.tight_layout(h_pad=0.05)
plt.savefig('ai_drive_unknown.png')

err_angle = gt_angle - pred_angle
err_dist = gt_to_middle_m - pred_to_middle_m
print("angle_err: min = %f, max = %f" % (err_angle.min(), err_angle.max()))
print("dist_err: min = %f, max = %f" % (err_dist.min(), err_dist.max()))