import os
import argparse
import numpy as np
from environment import Env
from agent import Agent
from lqr_agent import LQRAgent
from replay_memory import ReplayMemory
import matplotlib.pyplot as plt
import tensorflow as tf

parser = argparse.ArgumentParser()
parser.add_argument('--mode', type=str, default='play', help='modes={play, train}')
parser.add_argument('--num-steps', type=int, default=3300)
parser.add_argument('--ep-steps-limit', type=int, default=3300)
parser.add_argument('--warmup-steps', type=int, default=1000)
parser.add_argument('--memory-sz', type=int, default=1e6)
parser.add_argument('--learning-rate-actor', type=float, default=1e-4)
parser.add_argument('--learning-rate-critic', type=float, default=3e-4)
parser.add_argument('--discount', type=float, default=0.99)
parser.add_argument('--tau', type=float, default=1e-3)
parser.add_argument('--epsilon', type=float, default=1.0)
parser.add_argument('--epsilon-end', type=float, default=0.1)
parser.add_argument('--epsilon-endt', type=int, default=5e4)
parser.add_argument('--batch-sz', type=int, default=64)
parser.add_argument('--save-freq', type=int, default=10)
parser.add_argument('--seed', type=int, default=78940)
args = parser.parse_args()
print 'Start of DDPG_TORCS, mode is:', args.mode

state_dim = 3
action_dim = 1
writer_path = '../outputs/experiment4/summary'
saver_path = '../outputs/experiment4/model'
if not os.path.exists(writer_path):
    os.makedirs(writer_path)
if not os.path.exists(saver_path):
    os.makedirs(saver_path)

sess = tf.Session()
writer = tf.summary.FileWriter(writer_path)
print 'Creating agent, env and memory...'
agent = Agent(sess, state_dim, action_dim, writer)
lqr_agent = LQRAgent() # the LQRAgent must used with original Agent class to obtain cnn_laser
memory = ReplayMemory(args.memory_sz)
env = Env()
sess.run(tf.global_variables_initializer())

ft6 = 'pretrained/weights.hdf5'
ft8 = '/home/ld/tmp/keras.train.results/ppt_fig/mtl/ft8/weights.hdf5'
ft9 = '/home/lab/torcs/codebase/li_torcs/ddpg/laser/torcsnet.keras/pretrained/ft9/weights.hdf5'
if args.mode == 'play':
    agent.restore('pretrained/183337.ckpt')
    agent.torcsnet.load_weights(ft9)
    print 'Model is loaded'

episode = 0
step = 0
num_show = 30
cnn_angles = []
cnn_tms = []
true_angles = []
true_tms = []

# fig, axes = plt.subplots(nrows=2)
# axes[0].set_title('angle (green: true, red: pred)')
# axes[1].set_title('to_middle')
# axes[0].set_ylim(-0.08, 0.08)
# axes[1].set_ylim(-0.25, 0.25)
# axes[1].set_xlabel('time step')

# fig.show()
# fig.canvas.draw()

# x = np.arange(num_show)
# y = np.sin(x)*0
# def plot(ax, idx):
#     l1 = ax.plot(x, y, '-r', animated=True)[0]
#     l2 = ax.plot(x, y, '-g', animated=True)[0]
#     ax.legend(['pred', 'true'])
#     return [l1, l2]
# lines = [plot(ax, i) for i, ax in enumerate(axes)]
# backgrounds = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axes]

if args.mode == 'train':
    score_holder = tf.placeholder('float')
    score_summ = tf.summary.scalar('global/score', score_holder)


gt_to_middles_m = []
pred_to_middles_m = []
gt_angle = []
pred_angle = []
print 'Start of experiment...'
while step < args.num_steps:
    if np.mod(episode, 10) == 0:
        s, info = env.reset(relaunch=True)
    else:
        s, info = env.reset()

    term = False
    ep_rewards = 0.0
    ep_steps = 0
    epsilon = None

    while not term and ep_steps < args.ep_steps_limit:
        if args.mode == 'play':
            epsilon = 0  # disable exploration at play mode
        else:
            epsilon = max(args.epsilon_end, max(0.0, args.epsilon - step / args.epsilon_endt))

        a, q = agent.predict([s[0]], epsilon)
        cnn_laser = agent.cnn_predict_laser(s[1])
        cnn_laser.append(s[0][2]) # append speedX here
        cnn_a = agent.cnn_predict_action([np.array(cnn_laser)])
        # print ep_steps
        if ep_steps > 50:
            lqr_a = lqr_agent.predict_lqr_action(cnn_laser, s[0][2])
        else:
            lqr_a = 0.0

        

        gt_to_middles_m.append(info['dist_raced'])
        pred_to_middles_m.append(cnn_laser[1]*4.0)
        gt_angle.append(info['angle'])
        pred_angle.append(cnn_laser[0] * 3.1416)

        # cnn_angles.append(cnn_laser[0])
        # true_angles.append(s[0][0])
        # cnn_tms.append(cnn_laser[1])
        # true_tms.append(s[0][1])
        # angle_diff = cnn_laser[0] - s[0][0]
        # to_middle_diff = cnn_laser[1] - s[0][1]
        # a_diffs.append(angle_diff)
        # tm_diffs.append(to_middle_diff)
        # print 'cnn angle: %.4f, true angle: %.4f, diff: %.5f'%(cnn_laser[0], s[0][0], angle_diff)
        # print 'cnn to_middle: %.4f, true to_middle: %.4f, diff: %.5f'%(cnn_laser[1], s[0][1], to_middle_diff)


        # if ep_steps > 35:
        #     # subplot 1
        #     fig.canvas.restore_region(backgrounds[0])
        #     lines[0][0].set_ydata(np.array(cnn_angles[-num_show:]))
        #     lines[0][1].set_ydata(np.array(true_angles[-num_show:]))
        #     axes[0].draw_artist(lines[0][0])
        #     axes[0].draw_artist(lines[0][1])
        #     fig.canvas.blit(axes[0].bbox)

        #     # subplot 2
        #     fig.canvas.restore_region(backgrounds[1])
        #     lines[1][0].set_ydata(np.array(cnn_tms[-num_show:]))
        #     lines[1][1].set_ydata(np.array(true_tms[-num_show:]))
        #     axes[1].draw_artist(lines[1][0])
        #     axes[1].draw_artist(lines[1][1])
        #     fig.canvas.blit(axes[1].bbox)

        if ep_steps > 150:
            s2, r, term, info = env.step(cnn_a)
            # s2, r, term, info = env.step(a)            
        else:
            s2, r, term, info = env.step(0.0)
        # print 's: {}, r: {}'.format(s, r)
        if args.mode == 'train' and ep_steps > 24:
            memory.add(s, a, r, s2, term)

        if step > args.warmup_steps and args.mode == 'train':
            batch = memory.get_minibatch(args.batch_sz)
            agent.train(batch, args.learning_rate_actor, args.learning_rate_critic, step)
            agent.train_target()

        ep_rewards += r
        ep_steps += 1
        step += 1
        s = s2

    episode += 1
    print '# %d, steps: %d, ep_steps: %d, ep_r: %.4f, eps: %.4f' % \
                                         (episode+1, step, ep_steps, ep_rewards, epsilon)
    np.savez('ai_drive_dirt_2.npz', gt_to_middles_m = gt_to_middles_m,
                                pred_to_middles_m = pred_to_middles_m,
                                gt_angle = gt_angle,
                                pred_angle = pred_angle)
    print('stats info saved')

    if args.mode == 'train':
        summ = sess.run(score_summ, feed_dict={score_holder: ep_rewards})
        writer.add_summary(summ, global_step=step)
        writer.flush()

    if episode % args.save_freq == 0 and args.mode == 'train':
        path = saver_path + '/' + str(step) + '.ckpt'
        agent.save(path)

if args.mode == 'train':
    path = saver_path + '/final_' + str(step) + '.ckpt'
    agent.save(path)

env.end()
print 'Finish training'