"""
Load scenario

Usage:

python test_scenario.py --list  # show all scenarios
python test_scenario.py --scenario OtherLeadingVehicle_1 --reloadWorld --agent PATH_TO_AGENT
    e.g. python test_scenario.py --scenario FollowLeadingVehicle_1 --reloadWorld --agent=${ROOT_SCENARIO_RUNNER}/srunner/challenge/autoagents/HumanAgent.py 
python test_scenario.py --scenario OtherLeadingVehicle_1 --reloadWorld --agent PATH_TO_AGENT --waiting
"""

from scenario_runner import *
from srunner.scenariomanager.timer import GameTime
from srunner.challenge.envs.scene_layout_sensors import SceneLayoutReader, ObjectFinder
from srunner.challenge.envs.sensor_interface import CallBack, CANBusSensor, HDMapReader

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from srunner.tools.scenario_helper import generate_target_waypoint
from srunner.challenge.utils.route_manipulation import interpolate_trajectory
import threading
# import srunner.challenge.utils.route_configuration_parser as parser


# class GetRouteConfig(object):
#     def __init__(self, args):
#         if phase_codename == 'dev':
#             split_name = 'dev_split'
#             self.routes = '{}/srunner/challenge/routes_devtest.xml'.format(scenario_runner_root)
#             repetitions = 1
#         elif phase_codename == 'validation':
#             split_name = 'val_split'
#             self.routes = '{}/srunner/challenge/routes_testprep.xml'.format(scenario_runner_root)
#             repetitions = 3
#         elif phase_codename == 'test':
#             split_name = 'test_split'
#             self.routes = '{}/srunner/challenge/routes_testchallenge.xml'.format(scenario_runner_root)
#             repetitions = 3
#         else:
#             # debug mode
#             # using short routes
#             split_name = 'debug_split'
#             self.routes = '{}/srunner/challenge/routes_debug.xml'.format(scenario_runner_root)
#             repetitions = 1

#         # overwriting routes in case users passed their own
#         if args.routes:
#             self.routes = args.routes

#         # retrieve worlds annotations
#         world_annotations = parser.parse_annotations_file(args.scenarios)
#         # retrieve routes
#         route_descriptions_list = parser.parse_routes_file(self.routes)


class ScenarioTrainer(ScenarioRunner):
    def __init__(self, args):
        super(ScenarioTrainer, self).__init__(args)
        self.success_list = []
        if args.agent is not None:
            module_name = os.path.basename(args.agent).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.agent))
            self.module_agent = importlib.import_module(module_name)
        self.agent_instance = getattr(self.module_agent, self.module_agent.__name__)(args.config)
        self._sensors_list = []
        self.debug = 0
        self.timestamp = None
        self.frame_rate = 20

    def _cleanup(self, ego=False):
        """
        Remove and destroy all actors
        """

        CarlaDataProvider.cleanup()
        CarlaActorPool.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if ego:
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if hasattr(self, '_sensors_list'):
            for i, _ in enumerate(self._sensors_list):
                if self._sensors_list[i] is not None:
                    self._sensors_list[i].stop()
                    self._sensors_list[i].destroy()
                    self._sensors_list[i] = None
            self._sensors_list = []
        if self.agent_instance is not None:
            self.agent_instance.destroy()

    def _analyze_scenario(self, args, config):
        """
        Provide feedback about success/failure of a scenario
        """

        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        config_name = config.name
        if args.outputDir != '':
            config_name = os.path.join(args.outputDir, config_name)
        if args.junit:
            junit_filename = config_name + current_time + ".xml"
        filename = None
        if args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(args.output, filename, junit_filename):
            print("Success!")
            return True
        else:
            print("Failure!")
            return False

    def _load_and_wait_for_world(self, args, town, ego_vehicles=None):
        """
        Load a new CARLA world and provide data to CarlaActorPool and CarlaDataProvider
        """

        if args.reloadWorld:
            # self.world = self.client.load_world(town)
            # settings = self.world.get_settings()
            # settings.fixed_delta_seconds = 1.0 / self.frame_rate
            # self.world.apply_settings(settings)

            self.world = self.client.get_world()
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = 1.0 / self.frame_rate
            self.world.apply_settings(settings)

            self.world = self.client.load_world(town)
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self.frame_rate
            self.world.apply_settings(settings)

            self.world.on_tick(self._update_timestamp)
        else:
            # if the world should not be reloaded, wait at least until all ego vehicles are ready
            ego_vehicle_found = False
            if args.waitForEgo:
                while not ego_vehicle_found:
                    vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                    for ego_vehicle in ego_vehicles:
                        ego_vehicle_found = False
                        for vehicle in vehicles:
                            if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                                ego_vehicle_found = True
                                break
                        if not ego_vehicle_found:
                            print("Not all ego vehicles ready. Waiting ... ")
                            time.sleep(1)
                            break

        self.world = self.client.get_world()
        # self.world.on_tick(self._update_timestamp)
        CarlaActorPool.set_client(self.client)
        CarlaActorPool.set_world(self.world)
        CarlaDataProvider.set_world(self.world)

        # Wait for the world to be ready
        self.world.tick()

        if CarlaDataProvider.get_map().name != town:
            print("The CARLA server uses the wrong map!")
            print("This scenario requires to use map {}".format(town))
            return False

        return True

    def _update_timestamp(self, snapshot):
        self.timestamp = snapshot.timestamp

    def _load_and_run_scenario(self, args, config):
        """
        Load and run the scenario given by config
        """

        if not self._load_and_wait_for_world(args, config.town, config.ego_vehicles):
            self._cleanup()
            return

        # Prepare scenario
        print("Preparing scenario: " + config.name)
        try:
            CarlaActorPool.set_world(self.world)
            self._prepare_ego_vehicles(config.ego_vehicles, args.waitForEgo)
            if args.openscenario:
                scenario = OpenScenario(world=self.world,
                                        ego_vehicles=self.ego_vehicles,
                                        config=config,
                                        config_file=args.openscenario,
                                        timeout=100000)
            else:
                scenario_class = self._get_scenario_class_or_fail(config.type)
                scenario = scenario_class(self.world,
                                          self.ego_vehicles,
                                          config,
                                          args.randomize,
                                          args.debug)
        except Exception as exception:
            print("The scenario cannot be loaded")
            if args.debug:
                traceback.print_exc()
            print(exception)
            self._cleanup()
            return

        # Set the appropriate weather conditions
        weather = carla.WeatherParameters(
            cloudyness=config.weather.cloudyness,
            precipitation=config.weather.precipitation,
            precipitation_deposits=config.weather.precipitation_deposits,
            wind_intensity=config.weather.wind_intensity,
            sun_azimuth_angle=config.weather.sun_azimuth,
            sun_altitude_angle=config.weather.sun_altitude
        )

        self.world.set_weather(weather)

        # Create scenario manager
        self.manager = ScenarioManager(self.world, args.debug)

        # Load scenario and run it
        self.manager.load_scenario(scenario)

        spectator = self.world.get_spectator()
        ego_trans = self.ego_vehicles[0].get_transform()
        angle = ego_trans.rotation.yaw
        d = 6.4
        a = math.radians(180+angle)
        location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + ego_trans.location
        spectator.set_transform(carla.Transform(location, carla.Rotation(yaw=angle, pitch=-15)))

        if args.waiting:
            try:
                while True:
                    self.world.wait_for_tick()
            except:
                print("\nCancel waiting, start runing scenario")

        # gps_route, trajectory = generate_route(self.world, ego_trans.location)
        if self.agent_instance is not None:
            # self.agent_instance.set_global_plan(gps_route, trajectory)
            self.setup_sensors(self.agent_instance.sensors(), self.ego_vehicles[0])

        t1 = threading.Thread(target=self.manager.run_scenario)
        t2 = threading.Thread(target=self.run_agent)
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # Provide outputs if required
        success = self._analyze_scenario(args, config)

        # Stop scenario and _cleanup
        self.manager.stop_scenario()
        scenario.remove_all_actors()

        self._cleanup()
        return success

    def run_agent(self):
        while self.manager._running:
            GameTime.on_carla_tick(self.timestamp)
            CarlaDataProvider.on_carla_tick()
            ego_action = self.agent_instance()
            self.ego_vehicles[0].apply_control(ego_action)
            # self.world.tick()
            for _ in range(1):  # you can control action frequency here
                spectator = self.world.get_spectator()
                ego_trans = self.ego_vehicles[0].get_transform()
                angle = ego_trans.rotation.yaw
                d = 6.4
                a = math.radians(180+angle)
                location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + ego_trans.location
                spectator.set_transform(carla.Transform(location, carla.Rotation(yaw=angle, pitch=-15)))
                self.world.tick()

    def run_scenario_once(self, args):
        scenario_configurations = None
        scenario_config_file = ScenarioConfigurationParser.find_scenario_config(args.scenario, args.configFile)
        if scenario_config_file is None:
            print("Configuration for scenario {} cannot be found!".format(args.scenario))
            return

        scenario_configurations = ScenarioConfigurationParser.parse_scenario_configuration(scenario_config_file,
                                                                                           args.scenario)

        # Execute each configuration
        for config in scenario_configurations:
            success = self._load_and_run_scenario(args, config)
            self.success_list.append(success)

        self._cleanup(ego=(not args.waitForEgo))

        print("No more scenarios .... Exiting")
        return True

    def setup_sensors(self, sensors, vehicle):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :param vehicle: ego vehicle
        :return:
        """
        bp_library = self.world.get_blueprint_library()
        for sensor_spec in sensors:
            # These are the pseudosensors (not spawned)
            if sensor_spec['type'].startswith('sensor.scene_layout'):
                # Static sensor that gives you the entire information from the world (Just runs once)
                sensor = SceneLayoutReader(self.world)
            elif sensor_spec['type'].startswith('sensor.object_finder'):
                # This sensor returns the position of the dynamic objects in the scene.
                sensor = ObjectFinder(self.world, sensor_spec['reading_frequency'])
            elif sensor_spec['type'].startswith('sensor.can_bus'):
                # The speedometer pseudo sensor is created directly here
                sensor = CANBusSensor(vehicle, sensor_spec['reading_frequency'])
            elif sensor_spec['type'].startswith('sensor.hd_map'):
                # The HDMap pseudo sensor is created directly here
                sensor = HDMapReader(vehicle, sensor_spec['reading_frequency'])
            # These are the sensors spawned on the carla world
            else:
                bp = bp_library.find(str(sensor_spec['type']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', '5000')
                    bp.set_attribute('rotation_frequency', '20')
                    bp.set_attribute('channels', '32')
                    bp.set_attribute('upper_fov', '15')
                    bp.set_attribute('lower_fov', '-30')
                    bp.set_attribute('points_per_second', '500000')
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()

                # create sensor
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)
                sensor = self.world.spawn_actor(bp, sensor_transform,
                                                vehicle)
            # setup callback
            sensor.listen(CallBack(sensor_spec['id'], sensor, self.agent_instance.sensor_interface))
            self._sensors_list.append(sensor)

        # check that all sensors have initialized their data structure

        while not self.agent_instance.all_sensors_ready():
            if self.debug > 0:
                print(" waiting for one data reading from sensors...")
            self.world.tick()


def generate_route(world, vehicle_location, hop_resolution=1.0):
    # generate a route for current scenario
    # based on current scenario and map
    
    current_map = world.get_map()
    start_waypoint = current_map.get_waypoint(vehicle_location)

    # generate a dense route according to current scenario
    # could ref to <scenario_helper> module
    turn_flag = 0  # turn_flag by current scenario
    end_waypoint = generate_target_waypoint(start_waypoint, turn_flag)

    # generate a dense route
    # Setting up global router
    waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
    gps_route, trajectory = interpolate_trajectory(world, waypoints, hop_resolution)
    return gps_route, trajectory


# class SensorClass(object):
#     def __init__(self, world, args):
#         self.world = world
#         if args.agent is not None:
#             module_name = os.path.basename(args.agent).split('.')[0]
#             sys.path.insert(0, os.path.dirname(args.agent))
#             self.module_agent = importlib.import_module(module_name)
#         self.agent_instance = getattr(self.module_agent, self.module_agent.__name__)(args.config)
#         self._sensors_list = []
#         self.debug = 0

#     def setup_sensors(self, sensors, vehicle):
#         """
#         Create the sensors defined by the user and attach them to the ego-vehicle
#         :param sensors: list of sensors
#         :param vehicle: ego vehicle
#         :return:
#         """
#         bp_library = self.world.get_blueprint_library()
#         for sensor_spec in sensors:
#             # These are the pseudosensors (not spawned)
#             if sensor_spec['type'].startswith('sensor.scene_layout'):
#                 # Static sensor that gives you the entire information from the world (Just runs once)
#                 sensor = SceneLayoutReader(self.world)
#             elif sensor_spec['type'].startswith('sensor.object_finder'):
#                 # This sensor returns the position of the dynamic objects in the scene.
#                 sensor = ObjectFinder(self.world, sensor_spec['reading_frequency'])
#             elif sensor_spec['type'].startswith('sensor.can_bus'):
#                 # The speedometer pseudo sensor is created directly here
#                 sensor = CANBusSensor(vehicle, sensor_spec['reading_frequency'])
#             elif sensor_spec['type'].startswith('sensor.hd_map'):
#                 # The HDMap pseudo sensor is created directly here
#                 sensor = HDMapReader(vehicle, sensor_spec['reading_frequency'])
#             # These are the sensors spawned on the carla world
#             else:
#                 bp = bp_library.find(str(sensor_spec['type']))
#                 if sensor_spec['type'].startswith('sensor.camera'):
#                     bp.set_attribute('image_size_x', str(sensor_spec['width']))
#                     bp.set_attribute('image_size_y', str(sensor_spec['height']))
#                     bp.set_attribute('fov', str(sensor_spec['fov']))
#                     sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
#                                                      z=sensor_spec['z'])
#                     sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
#                                                      roll=sensor_spec['roll'],
#                                                      yaw=sensor_spec['yaw'])
#                 elif sensor_spec['type'].startswith('sensor.lidar'):
#                     bp.set_attribute('range', '5000')
#                     bp.set_attribute('rotation_frequency', '20')
#                     bp.set_attribute('channels', '32')
#                     bp.set_attribute('upper_fov', '15')
#                     bp.set_attribute('lower_fov', '-30')
#                     bp.set_attribute('points_per_second', '500000')
#                     sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
#                                                      z=sensor_spec['z'])
#                     sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
#                                                      roll=sensor_spec['roll'],
#                                                      yaw=sensor_spec['yaw'])
#                 elif sensor_spec['type'].startswith('sensor.other.gnss'):
#                     sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
#                                                      z=sensor_spec['z'])
#                     sensor_rotation = carla.Rotation()

#                 # create sensor
#                 sensor_transform = carla.Transform(sensor_location, sensor_rotation)
#                 sensor = self.world.spawn_actor(bp, sensor_transform,
#                                                 vehicle)
#             # setup callback
#             sensor.listen(CallBack(sensor_spec['id'], sensor, self.agent_instance.sensor_interface))
#             self._sensors_list.append(sensor)

#         # check that all sensors have initialized their data structure

#         while not self.agent_instance.all_sensors_ready():
#             if self.debug > 0:
#                 print(" waiting for one data reading from sensors...")
#             self.world.tick()


if __name__ == '__main__':

    DESCRIPTION = ("CARLA Scenario Runner: Setup, Run and Evaluate scenarios using CARLA\n"
                   "Current version: " + str(VERSION))

    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--host', default='127.0.0.1',
                        help='IP of the host server (default: localhost)')
    PARSER.add_argument('--port', default='2000',
                        help='TCP port to listen to (default: 2000)')
    PARSER.add_argument('--debug', action="store_true", help='Run with debug output')
    PARSER.add_argument('--output', action="store_true", help='Provide results on stdout')
    PARSER.add_argument('--file', action="store_true", help='Write results into a txt file')
    PARSER.add_argument('--junit', action="store_true", help='Write results into a junit file')
    PARSER.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')
    PARSER.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')
    PARSER.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    PARSER.add_argument('--additionalScenario', default='', help='Provide additional scenario implementations (*.py)')
    PARSER.add_argument('--reloadWorld', action="store_true",
                        help='Reload the CARLA world before starting a scenario (default=True)')
    # pylint: disable=line-too-long
    PARSER.add_argument(
        '--scenario', help='Name of the scenario to be executed. Use the preposition \'group:\' to run all scenarios of one class, e.g. ControlLoss or FollowLeadingVehicle')
    # pylint: enable=line-too-long
    PARSER.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')
    PARSER.add_argument('--repetitions', default=1, help='Number of scenario executions')
    PARSER.add_argument('--list', action="store_true", help='List all supported scenarios and exit')
    PARSER.add_argument('--listClass', action="store_true", help='List all supported scenario classes and exit')
    PARSER.add_argument('--openscenario', help='Provide an OpenSCENARIO definition')

    # ****************** add arguments to basic scenario_runner ******************
    PARSER.add_argument('--waiting', action="store_true", help='Just load scenario and then waiting')
    PARSER.add_argument("-a", "--agent", type=str, help="Path to Agent's py file to evaluate")
    PARSER.add_argument("--config", type=str, help="Path to Agent's configuration file", default="")
    # ****************************************************************************

    PARSER.add_argument('-v', '--version', action='version', version='%(prog)s ' + str(VERSION))
    ARGUMENTS = PARSER.parse_args()

    if ARGUMENTS.list:
        print("Currently the following scenarios are supported:")
        print(*ScenarioConfigurationParser.get_list_of_scenarios(ARGUMENTS.configFile), sep='\n')
        sys.exit(0)

    if ARGUMENTS.listClass:
        print("Currently the following scenario classes are supported:")
        print(*SCENARIOS.keys(), sep='\n')
        sys.exit(0)

    if not ARGUMENTS.scenario and not ARGUMENTS.openscenario:
        print("Please specify a scenario using '--scenario SCENARIONAME'\n\n")
        PARSER.print_help(sys.stdout)
        sys.exit(0)

    if not ARGUMENTS.agent:
        print("Please specify an agent using '--agent PATH_TO_YOUR_AGENT'\n\n")
        PARSER.print_help(sys.stdout)
        sys.exit(0)

    SCENARIOTRAINER = None
    try:
        SCENARIOTRAINER = ScenarioTrainer(ARGUMENTS)
        SCENARIOTRAINER.run_scenario_once(ARGUMENTS)
    finally:
        if SCENARIOTRAINER is not None:
            del SCENARIOTRAINER
