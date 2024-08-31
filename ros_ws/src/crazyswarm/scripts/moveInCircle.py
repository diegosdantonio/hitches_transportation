#!/usr/bin/env python

import numpy as np
import math
import time
from pycrazyswarm import Crazyswarm



def desired_pos(t, radius, t_nexpos, Z, offset):
    theta_circle = 2 * math.pi * t / t_nexpos + offset

    # position
    x = radius * math.sin(theta_circle)
    y = radius * math.cos(theta_circle)
    
    # Calculate the yaw
    yaw = math.atan2(-y, -x) 
    
    # update pos
    pos = np.array([x, y, Z])

    return pos, yaw

def move_in_circle(radius, Z, t_nexpos, time_values):

    for t in time_values:
        pos1, yaw1 = desired_pos(t, radius, t_nexpos, Z, 0)
        allcfs.crazyflies[0].goTo(pos1, yaw1, 3.)
        
        pos2, yaw2 = desired_pos(t, radius, t_nexpos, Z, math.pi)
        allcfs.crazyflies[1].goTo(pos2, yaw2, 3.)

        timeHelper.sleep(1.)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    Z = 1.0
    radius = 1.0
    t_nexpos = 6


    total_duration = 30.0  # Total duration for the circular motion
    num_steps = int(total_duration / t_nexpos)
    time_values = np.linspace(0, total_duration, 300)

    allcfs.takeoff(targetHeight=Z, duration=2.0 + Z)

    timeHelper.sleep(3.0)




    move_in_circle(radius, Z, t_nexpos, time_values)

    allcfs.land(targetHeight=0.02, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)
