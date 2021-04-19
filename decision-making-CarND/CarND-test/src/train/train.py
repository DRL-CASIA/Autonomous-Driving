# coding=utf-8
import socket  # socket模块
import json
import random
import numpy as np
from collections import deque
from keras.models import Sequential, Model
from keras.layers import Input, Dense, Conv2D, Flatten, concatenate
from keras.optimizers import Adam
from math import floor, sqrt
import tensorflow as tf
import subprocess
import time
import psutil
import pyautogui
import os
import pickle
from multiprocessing import Pool
from keras.backend.tensorflow_backend import set_session
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.3
set_session(tf.Session(config=config))
# import os
# os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
# os.environ["CUDA_VISIBLE_DEVICES"] = "4,5,6,7"


class DQNAgent:

    def __init__(self, state_height, state_width, action_size):
        self.state_height = state_height
        self.state_width = state_width
        self.action_size = action_size
        self.memory1 = deque(maxlen=20000)
        self.memory2 = deque(maxlen=20000)
        # self.memory3 = deque(maxlen=20000)
        self.gamma = 0.90    # discount rate
        self.epsilon = 1.0   # exploration rate
        self.epsilon_min = 0.3
        self.epsilon_decay = 1.0  # init with pure exploration
        self.learning_rate = 0.00025
        self.model = self._build_model()
        self.target_model = self._build_model()
        self.update_target_model()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        input1 = Input(shape=(1, self.state_height, self.state_width))
        conv1 = Conv2D(64, (4, 2), strides=1, activation='relu', padding='valid', data_format='channels_first',
                         input_shape=(1, self.state_height, self.state_width))(input1)
        conv2 = Conv2D(64, (4, 2), strides=1, activation='relu', padding='valid')(conv1)
        conv3 = Conv2D(3, 1, strides=1, activation='relu', padding='valid')(conv2)
        state1 = Flatten()(conv3)
        input2 = Input(shape=(3,))
        state2 = concatenate([input2, state1])
        state2 = Dense(256, activation='relu')(state2)
        state2 = Dense(64, activation='relu')(state2)
        out_put = Dense(self.action_size, activation='linear')(state2)
        model = Model(inputs=[input1, input2], outputs=out_put)
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))

        # model = Sequential()
        # model.add(Conv2D(64, (4, 2), strides=1, activation='relu', padding='valid', data_format='channels_first',
        #                  input_shape=(1, self.state_height, self.state_width)))
        # model.add(Conv2D(64, (4, 2), strides=1, activation='relu', padding='valid'))
        # model.add(Conv2D(3, 1, strides=1, activation='relu', padding='valid'))
        # model.add(Flatten())
        # model.add(Dense(256, activation='relu'))
        # model.add(Dense(64, activation='relu'))
        # model.add(Dense(self.action_size, activation='linear'))
        # model.compile(loss='mse',
        #               optimizer=Adam(lr=self.learning_rate))
        return model

    def update_target_model(self):
        # copy weights from model to target_model
        self.target_model.set_weights(self.model.get_weights())

    def remember1(self, state, action, reward, next_state):
        self.memory1.append((state, action, reward, next_state))

    def remember2(self, state, action, reward, next_state):
        self.memory2.append((state, action, reward, next_state))

    def remember3(self, state, action, reward, next_state):
        self.memory3.append((state, action, reward, next_state))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            print('random')
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch1 = random.sample(self.memory1, int(batch_size / 2))
        minibatch2 = random.sample(self.memory2, batch_size - int(batch_size / 2))
        minibatch = minibatch1 + minibatch2
        # for state, action, reward, next_state in minibatch:
        #     # target = reward
        #     target = (reward + self.gamma *
        #               np.amax(self.model.predict(next_state)[0]))
        #     target_f = self.model.predict(state)
        #     target_f[0][action] = target
        #     self.model.fit(state, target_f, epochs=1, verbose=0)
        for state, action, reward, next_state in minibatch:
            target = self.model.predict(state)
            t = self.target_model.predict(next_state)[0]
            target[0][action] = reward + self.gamma * np.amax(t)
            self.model.fit(state, target, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon = max(self.epsilon*self.epsilon_decay, self.epsilon_min)

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)


def connect(ser):
    conn, addr = ser.accept()  # 接受TCP连接，并返回新的套接字与IP地址
    print('Connected by', addr)  # 输出客户端的IP地址
    return conn


def open_ter(loc):
    os.system("gnome-terminal -e 'bash -c \"cd " + loc + " && ./path_planning; exec bash\"'")
    time.sleep(1)
    # return sim


def kill_terminal():
    pids = psutil.pids()
    for pid in pids:
        p = psutil.Process(pid)
        if p.name() == "gnome-terminal-server":
            os.kill(pid, 9)


def close_all(sim):
    if sim.poll() is None:
        sim.terminate()
        sim.wait()
    time.sleep(2)
    kill_terminal()


EPISODES = 100
location = "your_path_to/CarND-test/build"

HOST = '127.0.0.1'
PORT = 1234
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 定义socket类型，网络通信，TCP
server.bind((HOST, PORT))  # 套接字绑定的IP与端口
server.listen(1)  # 开始TCP监听

state_height = 45
state_width = 3
action_size = 3
agent = DQNAgent(state_height, state_width, action_size)
# agent.epsilon_min = 0.10
# agent.load("./train/episode30.h5")
# with open('./train/exp1.pkl', 'rb') as exp1:
#     agent.memory1 = pickle.load(exp1)
# with open('./train/exp2.pkl', 'rb') as exp2:
#     agent.memory2 = pickle.load(exp2)
batch_size = 16
episode = 1

while episode <= EPISODES:
    # 开启程序
    pool = Pool(processes=2)
    result = []
    result.append(pool.apply_async(connect, (server,)))
    pool.apply_async(open_ter, (location,))
    pool.close()
    pool.join()
    conn = result[0].get()
    sim = subprocess.Popen('your_path_to/term3_sim_linux/term3_sim.x86_64')
    time.sleep(2)
    pyautogui.click(x=1164, y=864, button='left')
    time.sleep(6)
    pyautogui.click(x=465, y=535, button='left')
    try:
        data = conn.recv(2000)  # 把接收的数据实例化
    except Exception as e:
        close_all(sim)
        continue
    while not data:
        try:
            data = conn.recv(2000)
        except Exception as e:
            close_all(sim)
            continue
    data = bytes.decode(data)
    # print(data)
    j = json.loads(data)

    # 初始化状态信息
    # Main car's localization Data
    # car_x = j[1]['x']
    # car_y = j[1]['y']
    car_s = j[1]['s']
    car_d = j[1]['d']
    car_yaw = j[1]['yaw']
    car_speed = j[1]['speed']
    # Sensor Fusion Data, a list of all other cars on the same side of the road.
    sensor_fusion = j[1]['sensor_fusion']
    grid = np.ones((51, 3))
    ego_car_lane = int(floor(car_d/4))
    grid[31:35, ego_car_lane] = car_speed / 100.0

    # sensor_fusion_array = np.array(sensor_fusion)
    for i in range(len(sensor_fusion)):
        vx = sensor_fusion[i][3]
        vy = sensor_fusion[i][4]
        s = sensor_fusion[i][5]
        d = sensor_fusion[i][6]
        check_speed = sqrt(vx * vx + vy * vy)
        car_lane = int(floor(d / 4))
        if 0 <= car_lane < 3:
            s_dis = s - car_s
            if -36 < s_dis < 66:
                pers = - int(floor(s_dis / 2.0)) + 30
                grid[pers:pers + 4, car_lane] = - check_speed / 100.0 * 2.237

    state = np.zeros((state_height, state_width))
    state[:, :] = grid[3:48, :]
    state = np.reshape(state, [-1, 1, state_height, state_width])
    pos = [car_speed / 50, 0, 0]
    if ego_car_lane == 0:
        pos = [car_speed / 50, 0, 1]
    elif ego_car_lane == 1:
        pos = [car_speed / 50, 1, 1]
    elif ego_car_lane == 2:
        pos = [car_speed / 50, 1, 0]
    pos = np.reshape(pos, [1, 3])
    # print(state)
    action = 0
    mess_out = str(action)
    mess_out = str.encode(mess_out)
    conn.sendall(mess_out)
    count = 0
    start = time.time()

    # 开始训练过程
    while True:
        # now = time.time()
        # if (now - start) / 60 > 15:
        #     close_all(sim)
        #     break
        try:
            data = conn.recv(2000)
        except Exception as e:
            pass
        while not data:
            try:
                data = conn.recv(2000)
            except Exception as e:
                pass
        data = bytes.decode(data)
        if data == "over":  # 此次迭代结束
            agent.save("./train/episode" + str(episode) + ".h5")
            print("weight saved")
            print("episode: {}, epsilon: {}".format(episode, agent.epsilon))
            with open('./train/train.txt', 'a') as f:
                f.write(" episode {} epsilon {}\n".format(episode, agent.epsilon))
            close_all(sim)
            conn.close()  # 关闭连接
            with open('./train/exp1.pkl', 'wb') as exp1:
                pickle.dump(agent.memory1, exp1)
            with open('./train/exp2.pkl', 'wb') as exp2:
                pickle.dump(agent.memory2, exp2)
            # with open('exp1.pkl', 'rb') as exp1:
            #     agent.memory1 = pickle.load(exp1)
            # with open('exp2.pkl', 'rb') as exp2:
            #     agent.memory2 = pickle.load(exp2)
            # with open('exp3.pkl', 'rb') as exp3:
            #     agent.memory3 = pickle.load(exp3)
            episode = episode + 1
            if episode == 41:
                agent.epsilon_min = 0.10
            if episode == 71:
                agent.epsilon_min = 0.03
            if episode == 6:
                agent.epsilon_decay = 0.99985  # start epsilon decay
            break
        try:
            j = json.loads(data)
        except Exception as e:
            close_all(sim)
            break

        # *****************在此处编写程序*****************
        last_state =
        last_pos =
        last_act =
        last_lane =
        # **********************************************

        # Main car's localization Data
        # car_x = j[1]['x']
        # car_y = j[1]['y']
        car_s = j[1]['s']
        car_d = j[1]['d']
        car_yaw = j[1]['yaw']
        car_speed = j[1]['speed']
        print(car_s)
        if car_speed == 0:
            mess_out = str(0)
            mess_out = str.encode(mess_out)
            conn.sendall(mess_out)
            continue
        # Sensor Fusion Data, a list of all other cars on the same side of the road.
        sensor_fusion = j[1]['sensor_fusion']
        ego_car_lane = int(floor(car_d / 4))
        if last_act == 0:
            last_reward = (2 * ((j[3] - 25.0) / 5.0))  # - abs(ego_car_lane - 1))
        else:
            last_reward = (2 * ((j[3] - 25.0) / 5.0)) - 10.0
        if grid[3:31, last_lane].sum() > 27 and last_act != 0:
            last_reward = -30.0
        grid = np.ones((51, 3))
        grid[31:35, ego_car_lane] = car_speed / 100.0
        # sensor_fusion_array = np.array(sensor_fusion)
        for i in range(len(sensor_fusion)):
            vx = sensor_fusion[i][3]
            vy = sensor_fusion[i][4]
            s = sensor_fusion[i][5]
            d = sensor_fusion[i][6]
            check_speed = sqrt(vx * vx + vy * vy)
            car_lane = int(floor(d / 4))
            if 0 <= car_lane < 3:
                s_dis = s - car_s
                if -36 < s_dis < 66:
                    pers = - int(floor(s_dis / 2.0)) + 30
                    grid[pers:pers + 4, car_lane] = - check_speed / 100.0 * 2.237
            if j[2] < -10:
                last_reward = float(j[2])  # reward -50, -100

        last_reward = last_reward / 10.0
        state = np.zeros((state_height, state_width))
        state[:, :] = grid[3:48, :]
        state = np.reshape(state, [-1, 1, state_height, state_width])
        # print(state)
        pos = [car_speed / 50, 0, 0]
        if ego_car_lane == 0:
            pos = [car_speed / 50, 0, 1]
        elif ego_car_lane == 1:
            pos = [car_speed / 50, 1, 1]
        elif ego_car_lane == 2:
            pos = [car_speed / 50, 1, 0]
        pos = np.reshape(pos, [1, 3])
        print("last_action:{}, last_reward:{:.4}, speed:{:.3}".format(last_act, last_reward, float(car_speed)))

        # agent.remember()
        # action = agent.act()
        # *****************在此处编写程序*****************
        if last_act != 0:
            agent.remember1(    )
        else:
            agent.remember2(    )
        action = agent.act(    )
        # **********************************************

        count += 1
        if count == 10:
            # *****************在此处编写程序*****************

            # **********************************************
            print("target model updated")
            count = 0

        if len(agent.memory1) > batch_size and len(agent.memory2) > batch_size:
            # *****************在此处编写程序*****************

            # **********************************************

        mess_out = str(action)
        mess_out = str.encode(mess_out)
        conn.sendall(mess_out)
