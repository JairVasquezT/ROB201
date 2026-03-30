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
    laser_dist = lidar.get_sensor_values()
    laser_angles = lidar.get_ray_angles()

    vect_goal = goal_pose[:2] - current_pose[:2]
    dist=np.linalg.norm(vect_goal)
    dir_goal=vect_goal/dist if dist > 0 else np.array([0, 0])
    theta = current_pose[2]
    k_goal=0.5
    k_rot=0.1
    force_x, force_y = 0.0, 0.0

    if dist > 80: 
        force_x=k_goal*(dir_goal[0]*np.cos(theta)+dir_goal[1]*np.sin(theta))
        force_y=k_goal*(-dir_goal[0]*np.sin(theta)+dir_goal[1]*np.cos(theta))

    if dist < 80:
        force_x = dist*(dir_goal[0]*np.cos(theta)+dir_goal[1]*np.sin(theta))/80*k_goal
        force_y = dist*(-dir_goal[0]*np.sin(theta)+dir_goal[1]*np.cos(theta))/80*k_goal

    d_safe=30 
    obs=(laser_dist<d_safe)&(laser_dist>0.1)
    dist_obs=laser_dist[obs]
    angle_obs=laser_angles[obs]
    mag=0
    f_rep_x=0
    f_rep_y=0
    k_rep=0.5
    if len(dist_obs)>0:
        print("Obstacle: ", dist_obs)
        print("Obstacle angle: ", angle_obs)
        obs_x = dist_obs*np.cos(angle_obs)
        obs_y = dist_obs*np.sin(angle_obs)

        #mag=(0.5/(dist_obs)**3)*(1/dist_obs-1/20)
        mag = (0.5/(dist_obs)**3)*(1/dist_obs - 1/d_safe)
        
        f_rep_x = -np.sum(mag * obs_x)*k_rep
        f_rep_y = -np.sum(mag * obs_y)*k_rep

    rotation_speed=0.1*np.arctan2(force_y+f_rep_y, force_x+f_rep_x)
    forward_speed=(force_x+f_rep_x)

    if dist < 5:
        print("Proche du but, ralentissement")
        forward_speed = 0
        rot_fin = goal_pose[2] - theta
        rot_fin = (rot_fin + np.pi) % (2 * np.pi) - np.pi
        rotation_speed = rot_fin
        
        if abs(rot_fin) < 0.05:
            rotation_speed = 0
            print("Goal reached")
            

    command = {"forward": forward_speed,
               "rotation": rotation_speed}

    return command




