"""
All information on README.md
"""

import tensorflow as tf
import tensorlayer as tl
from tensorlayer.layers import *
from gym_torcs import TorcsEnv
import numpy as np
import time
import matplotlib.pyplot as plt

img_dim = [64, 64, 3]
n_action = 1        # steer only (float, left and right 1 ~ -1)
steps = 5000        # maximum step for a game
batch_size = 32
n_epoch = 100

#没看懂，好像在计算操作
def get_teacher_action(ob):
    """ Compute steer from image for getting data of demonstration """
    steer = ob.angle*10/np.pi
    steer -= ob.trackPos*0.10
    return np.array([steer])

def img_reshape(input_img):
    """ (3, 64, 64) --> (64, 64, 3) """
    _img = np.transpose(input_img, (1, 2, 0))
    _img = np.flipud(_img)
    _img = np.reshape(_img, (1, img_dim[0], img_dim[1], img_dim[2]))
    return _img

images_all = np.zeros((0, img_dim[0], img_dim[1], img_dim[2]))
actions_all = np.zeros((0, n_action))
rewards_all = np.zeros((0,))

img_list = []
action_list = []
reward_list = []

###================= Get demonstration data
env = TorcsEnv(vision=True, throttle=False)
ob = env.reset(relaunch=True)

print("#"*50)
print('Collecting data from teacher (fake AI) ... ')
#收集示教数据
for i in range(1000):
    if i == 0:
        act = np.array([0.0])
    else:
        act = get_teacher_action(ob)
    if i % 100 == 0:
        print("step:", i)
    # if i > 50: # quick stop for quick debug
    #     break
    ob, reward, done, _ = env.step(act)
    img_list.append(ob.img)
    action_list.append(act)
    reward_list.append(np.array([reward]))

env.end()

print("#"*50)
print('Packing data into arrays... ')
for img, act, rew in zip(img_list, action_list, reward_list):
    images_all = np.concatenate([images_all, img_reshape(img)], axis=0)
    #横向拼接
    actions_all = np.concatenate([actions_all, np.reshape(act, [1,n_action])], axis=0)
    rewards_all = np.concatenate([rewards_all, rew], axis=0)
# save the teacher's data
tl.files.save_any_to_npy(save_dict={'im': images_all, 'act': actions_all, 're': rewards_all}, name='_tmp.npy')
# load the teacher's data
# data = tl.files.load_npy_to_any(name='_tmp.npy')
# images_all = data['im']; actions_all = data['act']; rewards_all = data['re']

# save some teacher's observaion
tl.files.exists_or_mkdir('image/teacher', verbose=True)
for i in range(0, len(images_all), 10):
    tl.vis.save_image(images_all[i], 'image/teacher/im_%d.png' % i)

###================= Define model
class Agent(object):
    def __init__(self, name='model', sess=None):
        #assert断言是声明其布尔值必须为真的判定，其返回值为假，就会触发异常。
        assert sess != None
        self.name = name
        self.sess = sess

        self.x = tf.placeholder(tf.float32, [None, img_dim[0], img_dim[1], img_dim[2]], name='Observaion')
        self.y = tf.placeholder(tf.float32, [None, n_action], name='Steer')

        self._build_net(True, False)
        self._build_net(False, True)
        self._define_train_ops()

        tl.layers.initialize_global_variables(self.sess)

        print()
        self.n_test.print_layers()
        print()
        self.n_test.print_params(False)
        print()
        # exit()

    def _build_net(self, is_train=True, reuse=None):
        with tf.variable_scope(self.name, reuse=reuse) as vs:
            tl.layers.set_name_reuse(reuse)

            '''
            Conv2d(net, n_filter=32, filter_size=(3, 3), strides=(1, 1), act=None,
                    padding='SAME', W_init=tf.truncated_normal_initializer(stddev=0.02),
                    b_init=tf.constant_initializer(value=0.0),
                    W_init_args={}, b_init_args={}, use_cudnn_on_gpu=None, data_format=None, name='conv2d', ):

            其中net值得就是该层的输入（该层的上一层）,n_filter官方说法是该层的滤波器（卷积核）的个数，n_filter是几的话该层就会输出多少个feature map
            3×3（filter_size）的卷积核，act 指定了一个激活函数
            '''

            n = InputLayer(self.x / 255, name='in') #在TensorLayer中每个神经网络的基础是一个 InputLayer 实例。它代表了将要提供给网络的输入数据。
            n = Conv2d(n, 32, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c1/1')
            n = Conv2d(n, 32, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c1/2')
            n = MaxPool2d(n, (2, 2), (2, 2), 'VALID', name='max1') #类似于2维卷积，只不过不做卷积操作，只取当前窗口最大值作为新图的像素值。
            #采样核size为2，stride为2，no padding。输入数据大小类似卷积层的计算方法，（input_width+2*pad-pool_size）/stride+1。
            # 前向传播中不仅要计算pool区域内的最大值，还要记录该最大值所在输入数据中的位置，目的是为了在反向传播中，需要把梯度值传到对应最大值所在的位置。

            n = DropoutLayer(n, 0.75, is_fix=True, is_train=is_train, name='drop1')
            #的第一个参数是输入层，第二个参数是保持概率（Keeping probability for the activation value），则数值不被置为零的概率。

            n = Conv2d(n, 64, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c2/1')
            n = Conv2d(n, 64, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c2/2')
            n = MaxPool2d(n, (2, 2), (2, 2), 'VALID', name='max2')
            # print(n.outputs)
            n = DropoutLayer(n, 0.75, is_fix=True, is_train=is_train, name='drop2')
            '''
            Convolution卷积层之后是无法直接连接Dense全连接层的，需要把Convolution层的数据压平（Flatten），然后就可以直接加Dense层了。
            也就是把 (height,width,channel)的数据压缩成长度为 height × width × channel 的一维数组，然后再与 FC层连接，这之后就跟普通的神经网络无异了。
            '''
            n = FlattenLayer(n, name='f')
            #Flatten层用来将输入“压平”，即把多维的输入一维化，常用在从卷积层到全连接层的过渡。Flatten不影响batch的大小。
            n = DenseLayer(n, 512, tf.nn.relu, name='dense1')
            #n_units 简明地给出了新全连接层的神经元单位数。act 指定了一个激活函数
            n = DropoutLayer(n, 0.5, is_fix=True, is_train=is_train, name='drop3')
            n = DenseLayer(n, n_action, tf.nn.tanh, name='o')

        if is_train:
            self.n_train = n
        else:
            self.n_test = n

    def _define_train_ops(self):
        self.cost = tl.cost.mean_squared_error(self.n_train.outputs, self.y, is_mean=False)
        self.train_params = tl.layers.get_variables_with_name(self.name, train_only=True, printable=False)
        self.train_op = tf.train.AdamOptimizer(learning_rate=0.0001).minimize(self.cost, var_list=self.train_params)

    def train(self, X, y, n_epoch=100, batch_size=10, print_freq=20):
        for epoch in range(n_epoch):
            start_time = time.time()
            total_err, n_iter = 0, 0
            for X_, y_ in tl.iterate.minibatches(X, y, batch_size, shuffle=True):
                _, err = self.sess.run([self.train_op, self.cost], feed_dict={self.x: X_, self.y: y_})
                total_err += err
                n_iter += 1
            if epoch % print_freq == 0:
                print("Epoch [%d/%d] cost:%f took:%fs" % (epoch, n_epoch, total_err/n_iter, time.time()-start_time))

    def predict(self, image):
        a = self.sess.run(self.n_test.outputs, {self.x : image})
        return a

    def save_model(self):
        tl.files.save_npz(self.n_test.all_params, name=self.name+'.npz', sess=self.sess)

    def load_model(self):
        tl.files.load_and_assign_npz(sess=self.sess, name=self.name+'.npz', network=self.n_test)

###===================== Pretrain model using data for demonstration
sess = tf.InteractiveSession()
model = Agent(name='model', sess=sess)
model.train(images_all, actions_all, n_epoch=n_epoch, batch_size=batch_size)
# save model after pretraining
model.save_model()
# model.load_model()
output_file = open('results.txt', 'w')

###===================== Aggregate and retrain
record_rewords=[]
episode_list=[]
n_episode = 50
for episode in range(n_episode):
    ob_list = []
    # restart the game for every episode
    env = TorcsEnv(vision=True, throttle=False)
    ob = env.reset(relaunch=True)
    reward_sum = 0.0
    print("#"*50)
    print("# Episode: %d start" % episode)
    for i in range(steps):
        act = model.predict(img_reshape(ob.img))
        ob, reward, done, _ = env.step(act)
        if done is True:
            break
        else:
            ob_list.append(ob)
        reward_sum += reward
        # print(i, reward, reward_sum, done, str(act[0]))
    print("# step: %d reward: %f " % (i, reward_sum))
    record_rewords.append(reward_sum)
    episode_list.append(episode)
    print("#"*50)
    output_file.write('episode:%02d\t Number of Steps: %02d\t Reward: %0.04f\n' % (episode,i, reward_sum))
    env.end()

    if i == (steps-1):
        break

    for ob in ob_list:
        images_all = np.concatenate([images_all, img_reshape(ob.img)], axis=0)
        # Dataset AGGregation: bring learner’s and expert’s trajectory distributions
        # closer by labelling additional data points resulting from applying the current policy
        actions_all = np.concatenate([actions_all, np.reshape(get_teacher_action(ob), [1, n_action])], axis=0)

    model.train(images_all, actions_all, n_epoch=n_epoch, batch_size=batch_size)
    model.save_model()

plt.figure('Line fig')
ax = plt.gca()
#设置x轴、y轴名称
ax.set_xlabel('x')
ax.set_ylabel('y')

#画连线图，以x_list中的值为横坐标，以y_list中的值为纵坐标
#参数c指定连线的颜色，linewidth指定连线宽度，alpha指定连线的透明度
ax.plot(episode_list, record_rewords, color='r', linewidth=1, alpha=0.6)

plt.show()

###=================== Play the game with the trained model
# while True:
#     env = TorcsEnv(vision=True, throttle=False)
#     ob = env.reset(relaunch=True)
#     reward_sum = 0.0
#     for i in range(steps):
#         act = model.predict(img_reshape(ob.img))
#         ob, reward, done, _ = env.step(act)
#         if done is True:
#             break
#         else:
#             ob_list.append(ob)
#         reward_sum += reward
#     print("PLAY WITH THE TRAINED MODEL")
#     print(reward_sum)
#     env.end()
