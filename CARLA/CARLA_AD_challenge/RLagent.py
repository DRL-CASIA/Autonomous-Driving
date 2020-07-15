import numpy as np
import torch
import carla
import copy
import datetime
from enum import Enum
from itertools import product
from collections import namedtuple
from srunner.challenge.envs.sensor_interface import SensorInterface
from srunner.scenariomanager.timer import GameTime
from srunner.challenge.utils.route_manipulation import downsample_route
from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track
from PIL import Image
# import carla.ColorConverter

Transition = namedtuple('Transition', ['state', 'action', 'reward', 'next_state'])

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# 由于每次Reset会删除当前Agent并重新产生，所以网络参数应与Agent脱离
# 此RL训练用Agent只负责与网络进行状态变量的预处理和传输以及动作的执行

class RLAgent(AutonomousAgent):
    def __init__(self, path_to_conf_file, episode_index):
        super(RLAgent, self).__init__(path_to_conf_file)
        #  RLagent net
        self.algorithm = None
        #  current global plans to reach a destination
        self._global_plan = None

        # this data structure will contain all sensor data
        self.sensor_interface = SensorInterface()

        self.state = None
        self.next_state = None
        self.reward = None

        self.image_shape = (3, 300, 200)
        self.action_space = []
        self.action_shape = None
        # self.steer_space = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]
        self.steer_space = [-1.0, -0.5, 0.0, 0.5, 1.0]
        self.acc_space = [0.0, 0.5, 1.0]

        # agent's initialization
        self.setup(path_to_conf_file)
        self.generate_action_space()

        self.index = 0

        self.episode_index = episode_index

    def setup(self, path_to_conf_file):
        """
        Initialize everything needed by your agent and set the track attribute to the right type:
            Track.ALL_SENSORS : LIDAR, cameras, GPS and speed sensor allowed
            Track.CAMERAS : Only cameras and GPS allowed
            Track.ALL_SENSORS_HDMAP_WAYPOINTS : All sensors and HD Map and waypoints allowed
            Track.SCENE_LAYOUT : No sensors allowed, the agent receives a high-level representation of the scene.
        """
        self.track = Track.CAMERAS

    def sensors(self):
        """
        Define the sensor suite required by the agent

        """
        sensors = [
            {'type':'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 300, 'height': 200, 'fov': 100, 'id': 'Mid'},
            # {'type':'sensor.camera.semantic_segmentation','x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            #  'width': 300, 'height': 200, 'fov': 100, 'id': 'Sem'}
        ]

        return sensors

    # 根据steer_num和acc_num划分离散动作空间
    def generate_action_space(self):
        for item in product(self.steer_space, self.acc_space):
            self.action_space.append(item)
        self.action_shape = len(self.action_space)

    def run_step(self, state):
        """
        Execute one step of navigation.
        :return: control
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        image = state['image']
        image = image.reshape(-1,*self.image_shape)
        image_tensor = torch.tensor(image)

        speedx = state['speedx']
        speedx_tensor = torch.tensor([speedx])
        speedx_tensor = speedx_tensor.unsqueeze(0)

        speedy = state['speedy']
        speedy_tensor = torch.tensor([speedy])
        speedy_tensor = speedy_tensor.unsqueeze(0)

        steer = state['steer']
        steer_tensor = torch.tensor([steer])
        steer_tensor = steer_tensor.unsqueeze(0)


        action_index = self.algorithm.select_action(image_tensor,speedx_tensor,speedy_tensor,steer_tensor)
        self.action = action_index
        control.steer = self.action_space[action_index][0]
        acc = self.action_space[action_index][1]

        # if acc >= 0.0:
        #     control.throttle = acc
        #     control.brake = 0.0
        # else:
        #     control.throttle = 0.0
        #     control.brake = abs(acc)
        control.throttle = acc
        print('throttle:',control.throttle)
        print('steer:',control.steer)
        
        return control

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        pass

    def __call__(self):
        timestamp = GameTime.get_time()
        wallclock = GameTime.get_wallclocktime()
        print('======[Agent] Wallclock_time = {} / Sim_time = {}'.format(wallclock, timestamp))
                   
        control = self.run_step(self.state)
        control.manual_gear_shift = False

        return control

    def all_sensors_ready(self):
        return self.sensor_interface.all_sensors_ready()

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):

        if self.track == Track.CAMERAS or self.track == Track.ALL_SENSORS:
            ds_ids = downsample_route(global_plan_world_coord, 32)

            self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1])
                                             for x in ds_ids]
            self._global_plan = [global_plan_gps[x] for x in ds_ids]

        else:   # No downsampling is performed

            self._global_plan = global_plan_gps
            self._global_plan_world_coord = global_plan_world_coord

    def get_image_shape(self):
        return self.image_shape

    def get_action_shape(self):
        return self.action_shape

    def set_algorithm(self, algorithm):
        self.algorithm = algorithm

    def get_state(self,speedx,speedy,steer):
        sensor_data = self.sensor_interface.get_data()
        # process image
        image_input = sensor_data['Mid']
        bgr_image = image_input[1][:,:,:3]

        # 注释掉此段，此段用来收集和处理语义分割图像，与现有挑战赛框架不符
        # semcamera = sensor_data['Sem']
        # sem_image = semcamera[1][:,:,:3]
        # self.save_image(50,'sem',sem_image)

        bgr_image = bgr_image[:,:,::-1]
        self.save_image(50,'rgb',bgr_image)

        bgr_image = bgr_image.astype(np.float32)
        bgr_image = np.multiply(bgr_image, 1.0 / 255.0)
        bgr_image = np.transpose(bgr_image,(2,1,0))


        self.index += 1

        if self.state is None:
            self.state = {'image':bgr_image , 'speedx':speedx , 'speedy':speedy , 'steer':steer}
        else:
            self.next_state = {'image':bgr_image , 'speedx':speedx , 'speedy':speedy , 'steer':steer}
            transition = Transition(self.state, self.action, self.reward, self.next_state)
            self.algorithm.store_transition(transition)
            self.state = self.next_state

    def get_reward(self, reward):
        self.reward = reward
        print('reward:',self.reward)

    def save_image(self , interval, path,image):
        if self.index % interval == 0: 
            im = Image.fromarray(image)
            im.save('/home/guoyoutian/scenario_runner-0.9.5/DQN/image/'+ path + '/' +str(self.episode_index) + '_%03d.jpeg' %(self.index/interval))
