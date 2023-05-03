#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import lcm
from bumblee import gps_command, action_command, collision_robots, platooning

from time import gmtime, strftime, sleep
import math
import numpy as np


spiral_id = 7

gps_x = 0
gps_y = 0
gps_p = 0
gps_q = 0
gps_id = 0


gps = gps_command()
gps.x = 0
gps.y = 0
gps.p = 0
gps.q = 0


action = action_command()
action.leftspeed = 0
action.rightspeed = 0

# cm
collison_radius = 10

robots_collis = collision_robots()
robots_collis.robots_len = 4
robots_collis.collision_array = [5, 2, 3, 12]


def distanceMetric(robot1, robot2):
    x_diff = robot1[0] - robot2[0]
    y_diff = robot1[1] - robot2[1]

    # calculate distance
    dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

    return dist


# center coordinates for referencing angles from
center_x = 314
center_y = 437


def angleMetric(robot1):
    # transform pixels to cm
    constX = float(center_x/2.5)
    constY = float(center_y/2.5)

    x_diff = float(robot1[0]-constX)
    y_diff = float(robot1[1]-constY)
    # calcuates angle : result is in rad
    angle = np.arctan2(x_diff, y_diff)
    return angle
