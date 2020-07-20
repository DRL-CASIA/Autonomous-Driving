import os
import operator
import argparse
import numpy as np
import tensorflow as tf
from agent import Agent
from environment import Env
from replay_memory import ReplayMemory

parser = argparse.ArgumentParser()
parser.add_argument('--mode', type=str, default='play', help='modes={play, train}')
parser.add_argument('--num-steps', type=int, default=5000)
parser.add_argument('--ep-steps-limit', type=int, default=5000)
parser.add_argument('--warmup-steps', type=int, default=2000)
parser.add_argument('--memory-sz', type=int, default=1e5)
parser.add_argument('--learning-rate-actor', type=float, default=1e-3)
parser.add_argument('--learning-rate-critic', type=float, default=1e-4)
parser.add_argument('--discount', type=float, default=0.99)
parser.add_argument('--tau', type=float, default=1e-3)
parser.add_argument('--epsilon', type=float, default=1.0)
parser.add_argument('--epsilon-end', type=float, default=0.1)
parser.add_argument('--epsilon-endt', type=int, default=1e6)
parser.add_argument('--batch-sz', type=int, default=64)
parser.add_argument('--save-freq', type=int, default=20000)
parser.add_argument('--seed', type=int, default=1221)
args = parser.parse_args()


print('Start of DDPG_TORCS, mode is:', args.mode)

state_dim = 3
action_dim = 1
writer_path = './results/summary'
saver_path = './results/model'
if not os.path.exists(writer_path):
    os.makedirs(writer_path)
if not os.path.exists(saver_path):
    os.makedirs(saver_path)

sess = tf.Session()
writer = tf.summary.FileWriter(writer_path)
print('Creating agent, env and memory...')

agent = Agent(sess, state_dim, action_dim, writer, args.learning_rate_actor, args.learning_rate_critic)
memory = ReplayMemory(args.memory_sz)
sess.run(tf.global_variables_initializer())
env = Env()

if args.mode == 'play':
    path = 'pre-trained/183337.ckpt'
    agent.restore(path)
    # agent.torcs_net.load('pre-trained/torcs_net.npy', sess)
    # print 'Model is restored from', path
    # print 'TORCS net is loaded'

episode = 0
step = 0



if args.mode == 'train':
    score_holder = tf.placeholder('float')
    score_summ = tf.summary.scalar('global/score', score_holder)


angle = []
to_track_middle = []
steer = []

print('Start of experiment...')
while step < args.num_steps:
    if np.mod(episode, 15) == 0:
        s = env.reset(relaunch=True)
    else:
        s = env.reset()

    term = False
    ep_rewards = 0.0
    ep_steps = 0
    epsilon = None
    steer_list = 4*[0.0]

    while not term and ep_steps < args.ep_steps_limit:
        # print('steps = %d, rewards = %f' % (ep_steps, ep_rewards))
        if args.mode == 'play':
            epsilon = 0  # disable exploration at play mode
        else:
            epsilon = max(args.epsilon_end, max(0.0, args.epsilon - step / args.epsilon_endt))

        if ep_steps > 0:
            a, q = agent.predict([s], epsilon)
            a = np.clip(a, -1.0, 1.0)

            s2, r, term, info = env.step(a)
            angle.append(info["angle"]*3.1415926)
            to_track_middle.append(info["to_middle_m"]) # abuse
            steer.append(a)
            print(("step = {}, angle = {}, dist = {}, steer = {}".format(ep_steps, angle[-1], to_track_middle[-1], steer[-1])))


            if args.mode == 'train' and ep_steps % 4 == 0:
                memory.add(s, a, r, s2, term)

            if step > args.warmup_steps and args.mode == 'train' and ep_steps % 4 == 0:
                batch = memory.get_minibatch(args.batch_sz)
                agent.train(batch, step)
                agent.train_target()
            ep_rewards += r
        else:
            s2, r, term, info = env.step(0.0)
        ep_steps += 1
        step += 1
        s = s2

    episode += 1
    print('# %d, steps: %d, ep_steps: %d, ep_r: %.4f, eps: %.4f' % \
            (episode+1, step, ep_steps, ep_rewards, epsilon))

    if args.mode == 'train':
        summ = sess.run(score_summ, feed_dict={score_holder: ep_rewards})
        writer.add_summary(summ, global_step=episode)
        writer.flush()

    if step % args.save_freq == 0 and args.mode == 'train':
        path = saver_path + '/iter_' + str(step) + '.ckpt'
        agent.save(path)

if args.mode == 'train':
    path = saver_path + '/final_' + str(step) + '.ckpt'
    agent.save(path)

env.end()
print('Finish training')
