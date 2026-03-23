""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1

    laser_dist = lidar.get_sensor_values()
    laser_angles = lidar.get_ray_angles()

    if np.min(laser_dist) < 10 and (laser_angles[np.argmin(laser_dist)] < 1.2 and laser_angles[np.argmin(laser_dist)] > -1.2):
        print("Obstacle detected at distance: ", np.min(laser_dist))
        print("Obstacle angle: ", laser_angles[np.argmin(laser_dist)])
        speed = 0.2*(10-np.min(laser_dist))/10
        if laser_angles[np.argmin(laser_dist)] > 0:
            rotation_speed = -0.5*(10-np.min(laser_dist))/10
        else: rotation_speed = 0.5*(10-np.min(laser_dist))/10
    else:
        speed = 0.2
        rotation_speed = 0.0

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    command = {"forward": 0,
               "rotation": 0}

    return command
