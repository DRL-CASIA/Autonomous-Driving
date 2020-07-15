import numpy as np
import matplotlib.pyplot as plt

# paths = ['drl_results/eroad/stats_1.npz',
#         #  'pid_results/eroad/stats_3.npz',
#          'lqr_results/eroad/stats_2.npz'
#          ]

# steers = []
# ag_errs = []
# d_errs = []
# for path in paths:
#     stats = np.load(path)
#     steers.append(stats['steers'])
#     ag_errs.append(stats['angle_errs'])
#     d_errs.append(stats['dist_errs'])

# data = [steers, ag_errs, d_errs]
# titles = ['steer', 'angle err', 'dist. err']
# legends = ['drl',
#         #    'pid',
#            'lqr']
# for i in range(len(data)):
#     plt.figure(i)
#     xs = data[i]
#     for x in xs:
#         plt.plot(x[::20])
#     plt.title(titles[i])
#     plt.legend(legends)
#     plt.show()

data = np.load("drl_statis_new.npz")
dist = data["to_track_middle"][:4500]
angle = data["angle"][:4500]
steer = data["steer"][:4500]

plt.subplot(311)
plt.plot(steer[::20])
plt.ylim(-0.5, 0.5)
plt.ylabel('Steer')

plt.subplot(312)
plt.plot(dist[::20])
plt.ylim(-1.0, 1.0)
plt.ylabel('Distance to middle (m)')

plt.subplot(313)
plt.plot(angle[::20])
plt.ylim(-0.3, 0.3)
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.tight_layout(h_pad=0.05)
plt.savefig("./drl_statis_new.png")
