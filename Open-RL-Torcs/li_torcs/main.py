import os
import argparse
import numpy as np
import tensorflow as tf
from agent import Agent
from environment import Env
from replay_memory import ReplayMemory

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
writer_path = './outputs/summary'
saver_path = './outputs/model'
if not os.path.exists(writer_path):
    os.makedirs(writer_path)
if not os.path.exists(saver_path):
    os.makedirs(saver_path)

sess = tf.Session()
writer = tf.summary.FileWriter(writer_path)
print 'Creating agent, env and memory...'

agent = Agent(sess, state_dim, action_dim, writer)
memory = ReplayMemory(args.memory_sz)
env = Env()
sess.run(tf.global_variables_initializer())

if args.mode == 'play':
    path = 'pre-trained/183337.ckpt'
    agent.restore(path)
    agent.torcs_net.load('pre-trained/torcs_net.npy', sess)
    print 'Model is restored from', path
    print 'TORCS net is loaded'

episode = 0
step = 0
num_show = 30
cnn_angles = []
cnn_tms = []
true_angles = []
true_tms = []

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
        print 'ep_step = ', ep_steps
        if args.mode == 'play':
            epsilon = 0  # disable exploration at play mode
        else:
            epsilon = max(args.epsilon_end, max(0.0, args.epsilon - step / args.epsilon_endt))

        a, q = agent.predict([s[0]], epsilon)
        cnn_laser = agent.cnn_predict_laser([s[1]])
        cnn_laser.append(s[0][2]) # append speedX here
        cnn_a = agent.cnn_predict_action([np.array(cnn_laser)])

        gt_to_middles_m.append(info['dist_raced'])
        pred_to_middles_m.append(cnn_laser[1]*4.0)
        gt_angle.append(info['angle'])
        pred_angle.append(cnn_laser[0] * 3.1416)

        cnn_angles.append(cnn_laser[0])
        true_angles.append(s[0][0])
        cnn_tms.append(cnn_laser[1])
        true_tms.append(s[0][1])
        angle_diff = cnn_laser[0] - s[0][0]
        to_middle_diff = cnn_laser[1] - s[0][1]
        # a_diffs.append(angle_diff)
        # tm_diffs.append(to_middle_diff)
        # print 'cnn angle: %.4f, true angle: %.4f, diff: %.5f'%(cnn_laser[0], s[0][0], angle_diff)
        # print 'cnn to_middle: %.4f, true to_middle: %.4f, diff: %.5f'%(cnn_laser[1], s[0][1], to_middle_diff)


        s2, r, term, info = env.step(cnn_a)
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
