"""
    Some function for developing DRL Agent training.

"""

import carla
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import distance_vehicle, draw_waypoints
from srunner.tools.scenario_helper import generate_target_waypoint
from srunner.challenge.utils.route_manipulation import interpolate_trajectory

import math
import os
import numpy as np
import collections
import weakref

def delete_vehicles(world):
    """
        delete all vehicles in selected world
    """
    actors = world.get_actors()
    for actor in actors:
        # if actor.type_id == vehicle.*
        id = actor.type_id
        actor_type = id.split(".")

        # destroy vehicle
        if actor_type[0] == "vehicle":
            actor.destroy()
        print("vehicles are deleted")

def set_spectator(vehicle):
    """
    Set spectator for a specified actor(vehicle)

    :param vehicle: selected actor
    :return: None
    """
    world = vehicle.get_world()
    # parameters for spectator:
    # behind distance - d, height - h
    d = 6.4
    h = 2.0

    ego_trans = vehicle.get_transform()
    angle = ego_trans.rotation.yaw
    a = math.radians(180 + angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), h) + ego_trans.location
    # set spector
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(location, carla.Rotation(yaw=angle, pitch=-15)))
    # print("spectator is set on ", vehicle.attributes['role_name'])

def generate_route(vehicle, turn_flag=0, hop_resolution=1.0):
    """
        Generate a local route for vehicle to next intersection
        todo: check if turn flag is unsuitable in next turn
    :param vehicle: Vehicle need route
    :param turn_flag: turn flag to
    :param hop_resolution: Distance between each waypoint
    :return: gps and coordinate route  generated
    """
    world = vehicle.get_world()
    map = world.get_map()

    # get initial location of ego_vehicle
    start_waypoint = map.get_waypoint(vehicle.get_location())

    # Using generate_target_waypoint to generate target waypoint
    # ref on scenario_helper.py module
    turn_flag = 0  # turn_flag by current scenario
    end_waypoint = generate_target_waypoint(start_waypoint, turn_flag)

    # generate a dense route according to current scenario
    # Setting up global router
    waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
    # from srunner.challenge.utils.route_manipulation import interpolate_trajectory
    gps_route, trajectory = interpolate_trajectory(world, waypoints, hop_resolution)
    return gps_route, trajectory

# get actor name
def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name



# ====================================================================================================
# Functions for kinematics and geometry calculation
# ====================================================================================================
# todo: merge these 2 funcitons, check class type and calculate distance
def distance_between_actor(vehicle, ped):
    """
    Compute distance between 2 actor, using <class actor> as input
    :param vehicle_transform:
    :param ped_transform:
    :return: Scalar distance
    """

    return  distance_to_vehicle(vehicle, ped.get_location())


def distance_to_vehicle(vehicle, target_location):
    """
    A quick fuction to calculate distance between a location and a vehicle
    :param vehicle: selected vehicle(class vehicle)
    :param target_location: desired location(class location)
    :return: distance to selected vehicle
    """
    # v_x = vehicle.get_location().x
    # v_y = vehicle.get_location().y
    # v_z = vehicle.get_location().z
    #
    dx = target_location.x - vehicle.get_location().x
    dy = target_location.y - vehicle.get_location().y
    # dz = target_location.z - vehicle.get_location().z

    distance_to_vehicle = math.sqrt(dx * dx + dy * dy)

    return distance_to_vehicle

def _get_rotation_matrix(transform):
    """
        Copied from CANbus sensor module and modified.
        Calculate a 3D transformation matrix in ZYX order.
    :param transform: transform of Origin
    :return: the rotation matrix
    """
    # caution: UE4 is using left-hand ortation order
    roll = np.deg2rad(-transform.rotation.roll)
    pitch = np.deg2rad(-transform.rotation.pitch)
    yaw = np.deg2rad(transform.rotation.yaw)
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    rotation_matrix = np.array([[cy * cp, -sy * sr + cy * sp * sr, cy * sp * cr + sy * sr],
                                [sy * cp, cy * sp * sr + cy * sr, -cy * sr + sy * sp * cr],
                                [-sp, cp * sr, cp * cr]])
    return rotation_matrix


def get_rotation_matrix_2D(transform):
    """
        Get a 2D transform matrix of a specified transform
        from actor reference frame to map coordinate frame
    :param transform: actor transform, actually only use yaw angle
    :return: rotation matrix
    """
    yaw = np.deg2rad(transform.rotation.yaw)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    rotation_matrix_2D = np.array([[cy, -sy],
                                   [sy, cy]])
    return rotation_matrix_2D


# function to plot local
def plot_local_coordinate_frame(world, origin_transform, axis_length_scale = 3):
    """
        Plot local coordinate frame.
        Using initial vehicle transform as origin
        todo: add text to identify axis, set x-axis always along longitudinal direction
    :param origin_transform: origin of local frame, in class transform
    :param axis_length_scale: length scale for axis vector
    :return: none, plot vectors of 3 axis in server world
    """

    # for test
    # origin_transform = transform
    # axis_length_scale = 3

    # longitudinal direction(x-axis)
    yaw = np.deg2rad(origin_transform.rotation.yaw)
    # x axis
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    Origin_coord = np.array(
        [origin_transform.location.x, origin_transform.location.y, origin_transform.location.z+1])
    # elevate z coordinate
    Origin_location = carla.Location(Origin_coord[0], Origin_coord[1], Origin_coord[2])
    # x axis destination
    x_des_coord = Origin_coord + axis_length_scale * np.array([cy, sy, 0])
    x_des = carla.Location(x_des_coord[0], x_des_coord[1], x_des_coord[2])
    # y axis destination
    y_des_coord = Origin_coord + axis_length_scale * np.array([-sy, cy, 0])
    y_des = carla.Location(y_des_coord[0], y_des_coord[1], y_des_coord[2])
    # z axis destination
    z_des_coord = Origin_coord + axis_length_scale * np.array([0, 0, 1])
    z_des = carla.Location(z_des_coord[0], z_des_coord[1], z_des_coord[2])

    """
    color set for each axis 
    x-axis red: (255, 0, 0)
    x-axis red: (0, 255, 0)
    x-axis red: (0, 0, 255)
    """
    x_axis_color = carla.Color(255, 0, 0)
    y_axis_color = carla.Color(0, 255, 0)
    z_axis_color = carla.Color(0, 0, 255)

    # axis feature
    # thickness = 0.1f
    # arrow_size = 0.1f

    # draw x axis
    world.debug.draw_arrow(Origin_location, x_des, color=x_axis_color)
    # draw y axis
    world.debug.draw_arrow(Origin_location, y_des, color=y_axis_color)
    # draw z axis
    world.debug.draw_arrow(Origin_location, z_des, color=z_axis_color)







