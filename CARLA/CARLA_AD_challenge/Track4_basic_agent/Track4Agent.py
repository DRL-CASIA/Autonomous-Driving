"""
python test_scenario.py --scenario FollowLeadingVehicle_12 \
--reloadWorld --agent=${ROOT_SCENARIO_RUNNER}/team_code/test_agent/Track4Agent.py \
--configFile FollowLeadingVehicle.xml
"""
import math
import numpy as np
import json
import scipy.misc

import carla

from autonomous_agent import AutonomousAgent, Track
from agents.navigation.local_planner import RoadOption

import controller2d
import configparser
import local_planner
import behavioural_planner


# Planning Constants
NUM_PATHS = 11
BP_LOOKAHEAD_BASE      = 15.0             # m
BP_LOOKAHEAD_TIME      = 1.5              # s
PATH_OFFSET            = 1.0              # m
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.5, 1.5, 1.5]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10
A_MAX                  = 1.5              # m/s^2
SLOW_SPEED             = 2.0              # m/s
STOP_LINE_BUFFER       = 3.5              # m
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 3                # Frequency divisor to make the
                                          # local planner operate at a lower
                                          # frequency than the controller
                                          # (which operates at the simulation
                                          # frequency). Must be a natural
                                          # number.
INTERP_DISTANCE_RES    = 0.01             # distance between interpolated points


def distance_vehicle(waypoint, vehicle_position):

    dx = waypoint['lat'] - vehicle_position[0]
    dy = waypoint['lon'] - vehicle_position[1]

    return math.sqrt(dx * dx + dy * dy)


def get_distance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def get_current_pose(transform):
    x = transform.location.x
    y = transform.location.y
    yaw = transform.rotation.yaw
    yaw = yaw * math.pi / 180.0
    if yaw > math.pi:
        yaw -= 2*math.pi
    elif yaw < -math.pi:
        yaw += 2*math.pi
    return x, y, yaw


def get_speed(current_pos, prev_pos, delta_time):
    dx = current_pos[0] - prev_pos[0]
    dy = current_pos[1] - prev_pos[1]
    # dz = current_pos[2] - prev_pos[2]
    # dis = math.sqrt(dx * dx + dy * dy + dz * dz)
    dis = math.sqrt(dx * dx + dy * dy)
    if delta_time == 0:
        return 0
    else:
        return dis / delta_time

def calculate_speed(v):
    speed = math.sqrt(v[0] * v[0] + v[1] * v[1])
    if speed < 1e-2:
        speed = 0
    return speed


class Track4Agent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.show_time = False
        self.track = Track.SCENE_LAYOUT
        # self.agent_engaged = False
        self.lp = local_planner.LocalPlanner(NUM_PATHS,
                                             PATH_OFFSET,
                                             CIRCLE_OFFSETS,
                                             CIRCLE_RADII,
                                             PATH_SELECT_WEIGHT,
                                             TIME_GAP,
                                             A_MAX,
                                             SLOW_SPEED,
                                             STOP_LINE_BUFFER)
        self.bp = behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE,
                                                    LEAD_VEHICLE_LOOKAHEAD)
        self.controller = None
        self.current_timestamp = 0.0
        self.prev_timestamp = 0.0
        self.count_frame = 0
        self.vehicles = None
        self._waypoints = None
        self.local_waypoints = None
        self.lead_car_speed = 0.0

        self.current_control = carla.VehicleControl()
        self.current_control.steer = 0.0
        self.current_control.throttle = 0.0
        self.current_control.brake = 1.0
        self.current_control.hand_brake = False

        # TODO: remove client and world, use sensor input data instead
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

    def sensors(self):
        """
        Define the sensor suite required by the agent
        :return: a list containing the required sensors in the following format:

        """
        sensors = [
                   {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
                   {'type': 'sensor.can_bus', 'reading_frequency': 25, 'id': 'can_bus'},
                   # {'type': 'sensor.scene_layout', 'id': 'scene_layout'},
                   {'type': 'sensor.object_finder', 'reading_frequency': 30, 'id': 'object_finder'},
                  ]

        return sensors

    def run_step(self, input_data, timestamp):
        # directions = self._get_current_direction(input_data['GPS'][1])
        if self._waypoints is None and self._global_plan is not None:
            self._waypoints = self._get_waypoints()
            self.controller = controller2d.Controller2D(self._waypoints)

        self.count_frame += 1
        delta_time = timestamp - self.prev_timestamp
        if delta_time == 0:
            return self.current_control, []
        self.prev_timestamp = self.current_timestamp
        self.current_timestamp = timestamp

        gnss_data = input_data['GPS'][1]
        can_bus_data = input_data['can_bus'][1]
        object_finder_data = input_data['object_finder'][1]
        current_transform = can_bus_data['transform']
        current_x, current_y, current_yaw = get_current_pose(current_transform)
        current_speed = can_bus_data['speed']

        lead_car_pos    = []
        # lead_car_length = []
        lead_car_speed  = []
        if self.vehicles is None:
            self.vehicles = object_finder_data['vehicles']
        for vehicle in object_finder_data['vehicles'].items():
            if vehicle[1]['id'] != object_finder_data['hero_vehicle']['id']:
                actor_id = vehicle[1]['id']
                for prev_vehicle in self.vehicles.items():
                    if actor_id == prev_vehicle[1]['id'] and vehicle[1]['location'][0] > 255:
                        lead_car_pos.append([vehicle[1]['location'][0], vehicle[1]['location'][1]])
                        # lead_car_speed.append(
                        #     get_speed(vehicle[1]['location'], prev_vehicle[1]['location'], delta_time))
                        lead_car_speed.append(calculate_speed(vehicle[1]['speed']))
                        # print(vehicle[1]['position'], prev_vehicle[1]['position'])
        self.vehicles = object_finder_data['vehicles']

        # TODO: get static obstacles from sensor data
        parkedcar_box_pts = []
        parkedcar_box_pts.append([254.0, 134.5])  # stationary object location

        self.bp._stopsign_fences = []
        best_path = []
        if self.count_frame % LP_FREQUENCY_DIVISOR == 0:
            self.count_frame = 1
            open_loop_speed = self.lp._velocity_planner.get_open_loop_speed(delta_time)
            print('open_loop_speed: {}'.format(open_loop_speed))
            ego_state = [current_x, current_y, current_yaw, open_loop_speed]
            self.bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)
            self.bp.transition_state(self._waypoints, ego_state, current_speed)

            if len(lead_car_pos) != 0:
                self.bp.check_for_lead_vehicle(ego_state, lead_car_pos[0])
            goal_state_set = self.lp.get_goal_state_set(self.bp._goal_index, self.bp._goal_state, self._waypoints, ego_state)
            paths, path_validity = self.lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)
            collision_check_array = self.lp._collision_checker.collision_check(paths, [parkedcar_box_pts])
            best_index = self.lp._collision_checker.select_best_path_index(paths, collision_check_array, self.bp._goal_state)
            if best_index == None:
                best_path = self.lp._prev_best_path
            else:
                best_path = paths[best_index]
                self.lp._prev_best_path = best_path
            # print(best_index)
            desired_speed = self.bp._goal_state[2]
            if len(lead_car_pos) >= 1:
                lead_car_state = [lead_car_pos[0][0], lead_car_pos[0][1], lead_car_speed[0]]
            else:
                lead_car_state = []
            decelerate_to_stop = self.bp._state == behavioural_planner.DECELERATE_TO_STOP
            self.local_waypoints = self.lp._velocity_planner.compute_velocity_profile(
                best_path, desired_speed, ego_state, current_speed, decelerate_to_stop, lead_car_state, self.bp._follow_lead_vehicle)

            if self.local_waypoints != None:
                wp_distance = []  # distance array
                self.local_waypoints_np = np.array(self.local_waypoints)
                for i in range(1, self.local_waypoints_np.shape[0]):
                    wp_distance.append(
                        np.sqrt((self.local_waypoints_np[i, 0] - self.local_waypoints_np[i - 1, 0]) ** 2 +
                                (self.local_waypoints_np[i, 1] - self.local_waypoints_np[i - 1, 1]) ** 2))
                wp_distance.append(0)  # last distance is 0 because it is the distance
                wp_interp = []  # interpolated values
                # (rows = waypoints, columns = [x, y, v])
                for i in range(self.local_waypoints_np.shape[0] - 1):
                    wp_interp.append(list(self.local_waypoints_np[i]))
                    num_pts_to_interp = int(np.floor(wp_distance[i] / \
                                                     float(INTERP_DISTANCE_RES)) - 1)
                    wp_vector = self.local_waypoints_np[i + 1] - self.local_waypoints_np[i]
                    wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                    for j in range(num_pts_to_interp):
                        next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
                        wp_interp.append(list(self.local_waypoints_np[i] + next_wp_vector))
                wp_interp.append(list(self.local_waypoints_np[-1]))
                self.controller.update_waypoints(wp_interp)
                pass
        # print(self.local_waypoints)
        if self.local_waypoints != None and self.local_waypoints != []:
            self.controller.update_values(current_x, current_y, current_yaw,
                                     current_speed, self.current_timestamp, self.count_frame)
            self.controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = self.controller.get_commands()
        else:
            cmd_throttle = 0.0
            cmd_steer = 0.0
            cmd_brake = 0.0

        safe_distance = BP_LOOKAHEAD_BASE + (BP_LOOKAHEAD_TIME - 0.5)* current_speed
        if get_distance(current_x, current_y, lead_car_pos[0][0], lead_car_pos[0][1]) < safe_distance:
            cmd_throttle = 0.0
            cmd_brake = 0.8

        print("******start showing input data******")
        print(current_x, current_y, current_yaw)
        print('lead_car_pos: {}'.format(lead_car_pos))
        print('lead_car_speed: {}'.format(lead_car_speed))
        # print(delta_time)
        # # print(self._waypoints)
        # print(gnss_data)
        # print(self.controller._desired_speed)
        print(cmd_throttle, cmd_steer, cmd_brake)
        print("*******end showing input data*******")

        self.current_control.steer = cmd_steer
        self.current_control.throttle = cmd_throttle
        self.current_control.brake = cmd_brake
        self.current_control.hand_brake = False

        return self.current_control, self.local_waypoints

    def _get_current_direction(self, vehicle_position):

        # for the current position and orientation try to get the closest one from the waypoints
        closest_id = 0
        min_distance = 100000
        for index in range(len(self._global_plan)):

            waypoint = self._global_plan[index][0]

            computed_distance = distance_vehicle(waypoint, vehicle_position)
            if computed_distance < min_distance:
                min_distance = computed_distance
                closest_id = index

        #print("Closest waypoint {} dist {}".format(closest_id, min_distance))
        direction = self._global_plan[closest_id][1]
        print ("Direction ", direction)
        if direction == RoadOption.LEFT:
            direction = 3.0
        elif direction == RoadOption.RIGHT:
            direction = 4.0
        elif direction == RoadOption.STRAIGHT:
            direction = 5.0
        else:
            direction = 2.0

        return direction

    def _get_waypoints(self):
        waypoints = []
        for index in range(len(self._global_plan_world_coord)):
            waypoint = self._global_plan_world_coord[index][0]
            # print(waypoint)
            # waypoints.append([waypoint['lat'], waypoint['lon'], waypoint['z']])
            waypoints.append([waypoint.location.x, waypoint.location.y, 10.0])
        # print(waypoints)
        return waypoints
