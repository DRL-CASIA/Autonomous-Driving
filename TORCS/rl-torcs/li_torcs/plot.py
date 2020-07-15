# -*- coding: utf8 -*-
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"]=["SimHei"]
plt.rcParams["axes.unicode_minus"]=False

'''AI Drive'''
cnn_ai_data = np.load('ai_drive_stats.npz')
mtl_ai_data = np.load('torcsnet.keras/ai_drive_stats_mtl.npz')
cnn_gt_angle = cnn_ai_data['gt_angle'].squeeze()
cnn_pred_angle = cnn_ai_data['pred_angle'].squeeze()
cnn_gt_to_middles_m = cnn_ai_data['gt_to_middles_m'].squeeze()
cnn_pred_to_middles_m = cnn_ai_data['pred_to_middles_m'].squeeze()

mtl_gt_angle = mtl_ai_data['gt_angle'].squeeze()
mtl_pred_angle = mtl_ai_data['pred_angle'].squeeze()
mtl_gt_to_middles_m = mtl_ai_data['gt_to_middles_m'].squeeze()
mtl_pred_to_middles_m = mtl_ai_data['pred_to_middles_m'].squeeze()

plt.figure(11)
plt.subplot(211)
# plt.plot(mtl_gt_to_middles_m[::20], color='#1f77b4', label = 'Groundtruth')
plt.plot(mtl_gt_to_middles_m[::20], color='#1f77b4', label = '真值')
# plt.plot(cnn_pred_to_middles_m[::20], '-.', color='#ff7f0e', label = 'Chen\'s method')
# plt.plot(mtl_pred_to_middles_m[::20], '--', color='#2ca02c', label = 'MTL')
plt.plot(mtl_pred_to_middles_m[::20], '--', color='#2ca02c', label = '预测值')
plt.xlim(-5, 165)
plt.ylim(-1.5, 1.5)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
           ncol=3, mode="expand", borderaxespad=0.)
# plt.xlabel('Time (s)')
plt.xlabel('时间 (s)')
# plt.ylabel('Distance to center (m)')
plt.ylabel('位置 (m)')


plt.subplot(212)
# plt.plot(cnn_gt_angle[::20], color='#1f77b4', label = 'Groundtruth')
plt.plot(cnn_gt_angle[::20], color='#1f77b4', label = '真值')
# plt.plot(cnn_pred_angle[::20], '-.', color='#ff7f0e', label = 'Chen\'s method')
# plt.plot(mtl_pred_angle[::20], '--', color='#2ca02c', label = ' MTL')
plt.plot(mtl_pred_angle[::20], '--', color='#2ca02c', label = ' 预测值')
plt.xlim(-5, 165)
plt.ylim(-0.15, 0.15)
# plt.xlabel('Time (s)')
plt.xlabel('时间 (s)')
# plt.ylabel('Yaw angle (rad)')
plt.ylabel('姿态 (rad)')
plt.tight_layout(h_pad=0.05)
plt.savefig('ai_drive.png')



# '''Bot Drive'''
# cnn_data = np.load('stats.npz')
# mtl_data = np.load('torcsnet.keras/stats_mtl.npz')
# cnn_gt_to_middles_m = cnn_data['gt_to_middles_m']
# cnn_pred_to_middles_m = cnn_data['pred_to_middles_m']
# cnn_gt_angle = cnn_data['gt_angle']
# cnn_pred_angle = cnn_data['pred_angle']

# mtl_gt_to_middles_m = mtl_data['gt_to_middles_m']
# mtl_pred_to_middles_m = mtl_data['pred_to_middles_m']
# mtl_gt_angle = mtl_data['gt_angle']
# mtl_pred_angle = mtl_data['pred_angle']

# # Plot distance error
# # all tracks
# plt.figure(1)
# plt.subplot(211)
# plt.plot(cnn_gt_to_middles_m[::20], color='#1f77b4', label = 'Groundtruth')
# plt.plot(cnn_pred_to_middles_m[::20], '-.', color='#ff7f0e', label = 'Chen\'s method')
# plt.xlim(-5, 165)
# plt.ylim(-1.5, 1.5)
# plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
#            ncol=2, mode="expand", borderaxespad=0.)
# plt.xlabel('Time (s)')
# plt.ylabel('Distance to center (m)')

# # Plot angle error
# plt.subplot(212)
# plt.plot(cnn_gt_angle[::20], color='#1f77b4')
# plt.plot(cnn_pred_angle.squeeze()[::20], '-.', color='#ff7f0e',)
# plt.xlim(-5, 165)
# plt.ylim(-0.15, 0.15)
# plt.xlabel('Time (s)')
# plt.ylabel('Yaw angle (rad)')
# plt.tight_layout(h_pad=0.05)
# plt.savefig('./cnn_drive.png')


# plt.figure(2)
# plt.subplot(211)
# plt.plot(mtl_gt_to_middles_m[::20],  color='#1f77b4', label = 'Groundtruth')
# plt.plot(mtl_pred_to_middles_m[::20], '--', color='#2ca02c', label = 'MTL-RL')
# plt.xlim(-5, 165)
# plt.ylim(-1.5, 1.5)
# plt.legend(bbox_to_anchor=(0., 1.02, 1., .12), loc=0,
#            ncol=2, mode="expand", borderaxespad=0.)
# plt.xlabel('Time (s)')
# plt.ylabel('Distance to center (m)')

# # Plot angle error
# plt.subplot(212)
# plt.plot(mtl_gt_angle[::20], color='#1f77b4',)
# plt.plot(mtl_pred_angle.squeeze()[::20], '--', color='#2ca02c',)
# plt.xlim(-5, 165)
# plt.ylim(-0.15, 0.15)
# plt.xlabel('Time (s)')
# plt.ylabel('Yaw angle (rad)')
# plt.tight_layout(h_pad=0.05)
# plt.savefig('./mtl_drive.png')



