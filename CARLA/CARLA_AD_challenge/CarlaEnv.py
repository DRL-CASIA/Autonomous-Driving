# import glob
# import os
# import sys

# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

import carla
# from carla import *

import random
import time
# import weakref
# import re
# import collections
import math
import traceback


class CarlaEnv(object):
    def __init__(self, carla_world):
        self.world = carla_world
        # settings = self.world.get_settings()
        # settings.fixed_delta_seconds = 0.1
        # self.world.apply_settings(settings)

        self.map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicles = self.blueprint_library.filter('vehicle.*')
        self.cars = [x for x in self.vehicles if int(x.get_attribute('number_of_wheels')) == 4]
        self.spawn_points = self.map.get_spawn_points()

        self.actor_list = []
        self.hero = None
        # self.restart()
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, ego_model='vehicle.lincoln.mkz2017', ego_random_location=False):
        # Spawn the ego car.
        while self.hero is None:
            self.setup_hero(ego_model, ego_random_location)

        # prepare the scenario
        for _ in range(0, 50):
            blueprint = random.choice(self.cars)
            blueprint.set_attribute('role_name', 'autopilot')
            transform = random.choice(self.spawn_points)
            # This time we are using try_spawn_actor. If the spot is already
            # occupied by another object, the function will return None.
            npc = self.world.try_spawn_actor(blueprint, transform)
            if npc is not None:
                self.actor_list.append(npc)
                npc.set_autopilot()
                # print('created %s' % npc.type_id)

    def reset(self, ticks=50):
        self.destroy()
        ob = None
        success, trans = self.setup_hero()
        if success:
            trans.location.x -= 10
            self.hero.set_transform(trans)
            # self.hero.set_transform(carla.Transform(trans.location - carla.Location(x=10)))
            for _ in range(ticks):
                self.world.wait_for_tick()
            ob, _, _ = self.get_state(self.hero)
        # self.hero.set_autopilot()
        # for _ in range(15):
        #     self.world.wait_for_tick()
        # self.hero.set_autopilot(False)
        return ob

    def setup_hero(self, model='vehicle.lincoln.mkz2017', random_location=False):
        blueprint = random.choice(self.vehicles.filter(model))
        blueprint.set_attribute('role_name', 'hero')
        self.spawn_points = self.map.get_spawn_points()
        if random_location:
            transform = random.choice(self.spawn_points)
        else:
            transform = self.spawn_points[348]
            # transform.location.x -= 10
            # waypoint = self.map.get_waypoint(transform.location)
            # transform = waypoint.transform
            # transform.location.z += 0.5
        if self.hero is None:
            self.hero = self.world.try_spawn_actor(blueprint, transform)
            if self.hero is not None:
                # self.actor_list.append(self.hero)
                # print('created ego car')
                return True, transform
            else:
                print('creat ego car failed')
                return False, transform
        else:
            self.hero.set_transform(transform)
            return True, transform

    def step(self, control, num_frames=10):
        for _ in range(num_frames):
            self.world.wait_for_tick()
            self.hero.apply_control(control)
            spectator = self.world.get_spectator()
            ego_trans = self.hero.get_transform()
            # print(ego_trans.rotation.yaw)
            angle = ego_trans.rotation.yaw
            d = 6.4
            a = math.radians(180+angle)
            location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + ego_trans.location
            spectator.set_transform(carla.Transform(location, carla.Rotation(yaw=angle, pitch=-15)))
        # control = carla.VehicleControl(throttle=0.0, steer=0, brake=1)
        # self.hero.apply_control(control)
        ob, reward, done = self.get_state(self.hero)
        return ob, reward, done

    def get_state(self, vehicle, show_message=False):
        if show_message:
            print("------------------------------>")

        dis, angle_delta, vx, vy = 1, 1, 0, 0
        reward = 0
        done = False

        trans = vehicle.get_transform()
        loc = trans.location
        rot = trans.rotation
        waypoint = self.map.get_waypoint(loc, project_to_road=True)
        if show_message:
            print("current lane id: {}".format(waypoint.lane_id))

        if waypoint.lane_id == -1:
            center = waypoint.get_right_lane()
            dis = loc.distance(center.transform.location)
            if dis > 5.3:
                dis = 5.3
                reward = -5.0
                done = True
            dis = -dis / 5.3
        elif waypoint.lane_id == -2:
            left = waypoint.get_left_lane()
            dis = loc.distance(left.transform.location)
            dis = dis - 3.5
            dis = dis / 5.3
        elif waypoint.lane_id == -3:
            center = waypoint.get_left_lane()
            dis = loc.distance(center.transform.location)
            dis = dis / 5.3
        elif waypoint.lane_id == -4:
            reward = -5.0
            done = True
        else:
            reward = 0.0
            done = True
        angle_delta = rot.yaw - waypoint.transform.rotation.yaw
        if angle_delta > 180:
            angle_delta -= 360
        elif angle_delta < -180:
            angle_delta += 360
        else:
            pass
        if show_message:
            print("angle delta: {:.3} degree".format(angle_delta))
        if angle_delta < -90:
            angle_delta = -90
            reward = -2.0 if reward > -2 else reward
            done = True
        elif angle_delta > 90:
            angle_delta = 90
            reward = -2.0 if reward > -2 else reward
            done = True
        normal_angle_delta = angle_delta / 90

        v_xy = vehicle.get_velocity()
        v = (3.6 * math.sqrt(v_xy.x**2 + v_xy.y**2 + v_xy.z**2))
        if show_message:
            print("current speed: {:.3} km/h".format(v))
        v = 60 if v > 60 else v
        v = v / 60
        angle_delta = angle_delta * math.pi / 180
        v_s = v * math.cos(angle_delta)
        v_d = v * math.sin(angle_delta)
        ob = [dis, normal_angle_delta, v_s, v_d]
        if reward >= 0 and done == False:
            reward = math.cos(angle_delta) - math.sin(abs(angle_delta)) - abs(dis)*1.5
        if show_message:
            print("<------------------------------")
        return ob, reward, done

    def destroy_hero(self):
        actors = [
            self.hero]
        for actor in actors:
            if actor is not None:
                actor.destroy()
        self.hero = None

    def destroy(self):
        # print('\ndestroying actors')
        self.destroy_hero()
        for actor in self.actor_list:
            actor.destroy()
        self.actor_list = []
        # print('done.')


class PlayGame(object):
    def __init__(self):
        self.world = None
        self.env = None
        
    @staticmethod
    def setup_world(host='localhost', fixed_delta_seconds=0.05):
        try:
            client = carla.Client(host, 2000)
            # client = carla.Client('localhost', 2000)

            world = client.get_world()
            town_map = world.get_map()
            if town_map.name != 'Town04':
                world = client.load_world('Town04')
                # town_map = world.get_map()

            world.wait_for_tick()
            settings = world.get_settings()
            # settings.synchronous_mode = True
            # settings.no_rendering_mode = False
            settings.fixed_delta_seconds = fixed_delta_seconds
            world.apply_settings(settings)

            # print(world.get_weather())
            world.set_weather(carla.WeatherParameters.ClearNoon)  # ClearNoon, ClearSunset, etc.
            # print(world.get_weather())
            print("world setup")

            return world, client

        except:
            traceback.print_exc()
            print("Setup world failed")
            return None

    def start(self):
        self.world, _ = self.setup_world()
        if self.world is None:
            return
        # self.world.tick()
        self.env = CarlaEnv(self.world)
        success, _ = self.env.setup_hero()
        # for _ in range(100):
        #     self.world.wait_for_tick()

        self.env.hero.set_autopilot()

        while True:
            # for _ in range(50):
            #     self.world.wait_for_tick()
            # ob, rw, done = self.env.get_state(self.env.hero)
            # print(ob, rw, done)
            self.world.wait_for_tick()
            spectator = self.world.get_spectator()
            ego_trans = self.env.hero.get_transform()
            # print(ego_trans.rotation.yaw)
            angle = ego_trans.rotation.yaw
            d = 6.4
            a = math.radians(180+angle)
            location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + ego_trans.location
            spectator.set_transform(carla.Transform(location, carla.Rotation(yaw=angle, pitch=-15)))

            # env.hero.apply_control(control)

    def run(self, train_indicator=0):    #1 means Train, 0 means simply Run
        pass


def main():
    try:
        play_game = PlayGame()
        play_game.start()
    except:
        traceback.print_exc()
        if play_game:
            if play_game.env is not None:
                play_game.env.destroy()
    finally:
        del play_game


if __name__ == '__main__':

    main()
