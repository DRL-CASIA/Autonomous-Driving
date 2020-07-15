import numpy as np
import matplotlib.pyplot as plt


mtl_data = np.load('ai_drive_stats_new1.npz')
mtl_gt_to_middles_m = mtl_data['gt_to_middles_m']
mtl_pred_to_middles_m = mtl_data['pred_to_middles_m']
mtl_gt_angle = mtl_data['gt_angle']
mtl_pred_angle = mtl_data['pred_angle']
gt_dist = mtl_gt_to_middles_m[::]
pred_dist = mtl_pred_to_middles_m[::]
gt_angle = mtl_gt_angle[::]
pred_angle = mtl_pred_angle.squeeze()[::]

err_dist = pred_dist - gt_dist
err_angle = pred_angle - gt_angle

err_dist = err_dist.tolist()
err_dist = [abs(e) for e in err_dist]
print("mean err_dist = %f, max = %f" % (np.mean(err_dist), np.max(err_dist)))

err_angle = err_angle.tolist()
err_angle = [abs(e) for e in err_angle]
print("mean err_angle = %f, max = %f" % (np.mean(err_angle), np.max(err_angle)))

plt.figure()
plt.subplot(211)
plt.plot(mtl_gt_to_middles_m[::20],  color='#1f77b4', label = 'Groundtruth')
plt.plot(mtl_pred_to_middles_m[::20], '--', color='#2ca02c', label = 'MTL-RL')
plt.xlim(-5, 165)
plt.ylim(-1.5, 1.5)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
           ncol=2, mode="expand", borderaxespad=0.)
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')

# Plot angle error
plt.subplot(212)
plt.plot(mtl_gt_angle[::20], color='#1f77b4',)
plt.plot(mtl_pred_angle.squeeze()[::20], '--', color='#2ca02c',)
plt.xlim(-5, 165)
plt.ylim(-0.15, 0.15)
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (rad)')
plt.tight_layout(h_pad=0.05)
plt.show()