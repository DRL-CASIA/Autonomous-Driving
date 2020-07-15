"""
A developing version...

Try to create an environment in gym style.

todo evaluate with stable_baselines
"""

# import sumo
import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

import traci
import sumolib
# from sumolib import checkBinary

import numpy as np
import heapq

# import gym module
import gym
from gym import spaces
# from gym import Env


# vehicle color
# magenta = (255, 0, 255)
# cyan = (0, 255, 255)
# yellow = (255,255,0)


class SUMOEnv(gym.Env):
    """
    A gym style env for sumo junction turning experiment.
    """

    action_space = spaces.Discrete(4)
    observation_space = spaces.Box(low=np.array([[-1000., -1000., -1., -1., 0.]*6]),
                                   high=np.array([[1000., 1000., 1., 1., 20.]*6]),
                                   shape=(1, 30))

    reward_range = (-float('inf'), float('inf'))

    # route settings
    route_selection = ['left', 'mid', 'right']  # defined by rou.xml file
    # route = route_selection[2]

    # duration time of slowDown method
    time_interval = .1

    # time limit of a complete episode
    time_limit = 25.

    def __init__(self, config_file=None, with_gui=False):

        # set path
        if config_file is None:
            config_file = '/home/lyq/PycharmProjects/gym-sumo/config/basic/basic.sumocfg'

        # start sumo
        self.init_simulator(config_file, with_gui)

        # attributes for RL module
        self.state = None
        self.action = None
        self.reward = None

        # flags to identify task of ego vehicle
        self.reach_goal_flag = False
        self.collision_flag = False

        self.sumo_step = 0  # total step of a episode

        # count result
        self.result_count = {'success': 0, 'collision': 0, 'time_exceed': 0}

        self.vehicle_list = []
        self.route = self.route_selection[2]

        self.start_time = None
        self.end_time = None
        self.episode_time = None

    def init_simulator(self, cfg_file, with_gui=True, port_num=9000):
        """
        Initialize simulator with default parameters.

        todo add API to set config file and port(how to use??)

        :param config:
        :param with_gui:
        :param port_num:
        :return:
        """
        if not cfg_file:
            cfg_file = '../config/test.sumocfg'

        if with_gui:
            sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            sumo_binary = sumolib.checkBinary('sumo')

        """
        --collisions.check-junctions is supposed to set as 'true'

        According to sumo docs:
        When setting the option --collisions.check-junctions, 
        collisions between vehicles on the same intersection are 
        also checked by detecting overlap of the vehicles shapes.

        --collision.action is supposed to set as 'warn'
        When set to 'warn', collision vehicles are stored in list, 
        but won't be removed.
        The simulation will not be interrupted.

        --collision.stoptime should not be set.        
        """

        # warn is suggested to use
        traci.start([sumo_binary, '-c', cfg_file,
                     "--collision.check-junctions", 'true',
                     "--collision.action", 'warn'])

        # traci.start([sumo_binary, '-c', cfg_file])

        print('sumo simulation start.')

    def init_traffic_flow(self):
        """
        Init a traffic flow before an episode start.

        2 conditions to identify traffic flow is ready.
        """

        # check enough npc vehicles
        target_number = 10

        # get TLS target phase for different route
        if self.route in ['mid', 'left']:
            target_phase = 0  # int(0) indicates WE flow green light
        else:
            target_phase = 2  # int(2) indicates NS flow green light

        vehNum_cond = False
        tls_cond = False
        conditions = [vehNum_cond, tls_cond]

        while not all(conditions):
            # tick the simulator before check conditions
            traci.simulationStep()
            # update vehicle number condition
            vehicle_numbers = len(traci.vehicle.getIDList())
            vehNum_cond = vehicle_numbers >= target_number
            # update TLS condition
            phase = traci.trafficlight.getPhase('99810003')
            tls_cond = phase is target_phase

            conditions = [vehNum_cond, tls_cond]

            # if not conditions:
            #     print('d')
            #
            # print('d')

        print('traffic flow is ready')

    @staticmethod
    def get_single_vehicle_state(vehicle_id):
        """
        Get state of a single vehicle.
        :param vehicle_id: id of the vehicle
        :return: state in list
        """

        # coordinates of ego vehicle
        x, y = traci.vehicle.getPosition(vehicle_id)
        yaw = np.deg2rad(traci.vehicle.getAngle(vehicle_id))

        # sumo uses different kinetics
        # todo check definition of lateral speed in sumo
        speed = traci.vehicle.getSpeed(vehicle_id)
        # state representation for RL
        state = [x, y, np.cos(yaw), np.sin(yaw), speed]

        return state

    def get_near_npc(self):
        """
        Select a certain number of nearest npc vehicles for state representation.
        :return: near npc vehicle(id) list
        """
        near_npc_number = 5  # required npc number

        # location of ego vehicle
        ego_loc = traci.vehicle.getPosition('ego')
        ego_loc = np.array(ego_loc)

        # list to store distance of each npc vehicle
        distance_list = []

        # update vehicle list
        self.vehicle_list = traci.vehicle.getIDList()

        # Get a dict to describe npc vehicle location and distance
        for index, vehicle in enumerate(self.vehicle_list):
            if vehicle == 'ego':
                continue

            loc = traci.vehicle.getPosition(vehicle)
            loc = np.array(loc)

            distance = np.linalg.norm(loc - ego_loc)

            add_dict = {'index': index, 'vehicle_id': vehicle, 'distance': distance}
            distance_list.append(add_dict)

        # get nearest npc vehicles' ID
        _near_npc = heapq.nsmallest(near_npc_number, distance_list, key=lambda s: s['distance'])
        near_npc_list = [x['vehicle_id'] for x in _near_npc]

        # set color for npc vehicles
        for veh in self.vehicle_list:
            if veh == 'ego':
                continue
            # npc vehicle color
            if veh in near_npc_list:
                traci.vehicle.setColor(veh, (255, 0, 255))  # Magenta
            else:
                traci.vehicle.setColor(veh, (255, 255, 0))  # yellow

        return near_npc_list

    def get_state(self):
        """
        Get current state of current time step

        todo check how to get npc vehicles of each route
        todo check if npc vehicle number is less than desired
        # if len(near_npc_list) >= 5:
        #     print('npc vehicle amount is larger than 5.')

        :return: state(list)
        """
        vehicle_list = traci.vehicle.getIDList()
        if self.check_ego_exist():
            state = self.get_single_vehicle_state('ego')
            # self.get_near_npc() method will use ego vehicle
            for veh in self.get_near_npc():
                state += self.get_single_vehicle_state(veh)
            return state
        else:
            print("Fail to get state correctly.")
            return None

    def check_ego_exist(self):
        """"""
        exist_flag = False
        self.vehicle_list = traci.vehicle.getIDList()
        for veh in self.vehicle_list:
            if veh == 'ego':
                exist_flag = True
        return exist_flag

    def reset(self):
        """
        Reset simulation for a new episode.
        """
        # remove ego vehicle
        if self.check_ego_exist():
            traci.vehicle.remove(vehID="ego")  # default reason=3
            traci.simulationStep()
            print('ego vehicle is removed')
        else:
            print('ego vehicle is not found.')

        # set traffic flow
        self.init_traffic_flow()

        # spawn ego vehicle
        try:
            traci.vehicle.add(vehID="ego", routeID=self.route, typeID="ego", departLane='0', departPos=165.)
            # set speed mode
            traci.vehicle.setSpeedMode('ego', 6)
            speed_mode = traci.vehicle.getSpeedMode('ego')
            # print('Speed mode of ego is: ', speed_mode)

            traci.simulationStep()
            if self.check_ego_exist():
                print('ego vehicle is spawned.')
        except:
            print('Fail to spawn ego vehicle')

        # store start time of this episode
        self.start_time = traci.simulation.getTime()

        # reset flags
        self.reach_goal_flag = False
        self.collision_flag = False

        # initial state of a episode
        state = self.get_state()
        observation = np.array([state])

        return observation

    def compute_reward(self):
        """
        Get reward of current step.

        todo add max time(time step) critics
        :return: reward value(scalar), done flag, aux info
        """
        reward = 0.
        done = False
        aux = 'running'

        # Step cost
        reward += -0.15

        # check collision
        collision_list = traci.simulation.getCollidingVehiclesIDList()

        # check time exceed
        elapsed_time = traci.simulation.getTime() - self.start_time
        time_exceed = elapsed_time >= self.time_limit

        if collision_list:
            for veh in collision_list:
                if veh == 'ego':
                    print('Ego vehicle collides with npc vehicle.')
                    self.collision_flag = True
                    self.result_count['collision'] += 1
                    done = True
                    reward += -500
                    aux = 'collision'
                    return reward, done, aux
        elif time_exceed:
            # check if beyond max time limit
            self.result_count['time_exceed'] += 1
            done = True
            reward += -50
            aux = 'time_exceed'
        else:
            # check if reach goal
            # ego_lane = traci.vehicle.getLaneID('ego')  # get lane that ego vehicle is on
            ego_route = traci.vehicle.getRoute('ego')
            ego_road = traci.vehicle.getRoadID('ego')

            if ego_road == ego_route[-1]:
                if traci.vehicle.getLanePosition('ego') >= 15.:
                    print('Success. Ego vehicle reached goal.')
                    self.reach_goal_flag = True
                    self.result_count['success'] += 1
                    done = True
                    reward += 50
                    aux = 'success'

        if done:
            self.end_time = traci.simulation.getTime()
            self.episode_time = self.end_time - self.start_time

        # neither collide and reach goal
        return reward, done, aux

    def step(self, action):
        """
        The step method inherits from gym.Env.step.
        Take a time step of simulation.
        Input args action is index of action space

        todo use gym action class
        :param action:
        :return: state, reward, done, {}
        """

        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        target_speed = [0, 3, 6, 9]

        # apply control to ego vehicle
        if self.check_ego_exist():
            traci.vehicle.slowDown('ego', target_speed[action], self.time_interval)
            traci.simulationStep()
        else:
            print('Fail to apply ego vehicle action.')

        # CAUTION: compute reward before get new state
        # get reward of this step
        reward, done, info = self.compute_reward()
        aux = {'exp_state': info}
        state = self.get_state()

        # # update state
        # # todo check if ego vehicle exist
        # if not done:
        #     # get state of ego and npc vehicle
        #     state = self.get_single_vehicle_state('ego')
        #     for veh in self.near_npc_list:
        #         state += self.get_single_vehicle_state(veh)

        # check if episode is done
        # done = self.check_status()

        observation = np.array([state])

        # last empty dict is supposed to contain auxiliary diagnostic information
        return observation, reward, done, aux


if __name__ == '__main__':
    # config_file = '../config/test.sumocfg'
    # env = SUMOEnv(config_file)

    env = SUMOEnv()

    print('d')

    # ==================================================
    # test one step of simulation
    # ==================================================

    # env.init_traffic_flow()

    # env.set_ego_vehicle()

    # env.get_state()

    print('d')
