#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

object_height = 1.
targetPos = np.array([0, 0, 0])
num_layers = 4

def swapPosition_twice(cf1, cf2, cf3, cf4, height_gap, swapDuration):
    pos1 = np.array(cf1.position())
    pos2 = np.array(cf2.position())
    pos3 = np.array(cf3.position())
    pos4 = np.array(cf4.position())

    # Initialize positions
    init_pos1_H = pos1 + np.array([0, 0, height_gap]) 
    init_pos1_L = pos1 + np.array([0, 0, -height_gap])

    init_pos2_H = pos2 + np.array([0, 0, height_gap]) 
    init_pos2_L = pos2 + np.array([0, 0, -height_gap])
    
    init_pos3_H = pos3 + np.array([0, 0, height_gap]) 
    init_pos3_L = pos3 + np.array([0, 0, -height_gap])

    init_pos4_H = pos4 + np.array([0, 0, height_gap]) 
    init_pos4_L = pos4 + np.array([0, 0, -height_gap])
    

    # Initial positions
    cf1.goTo(init_pos1_H, 0, swapDuration)
    cf2.goTo(init_pos2_L, 0, swapDuration)
    cf3.goTo(init_pos3_H, 0, swapDuration)
    cf4.goTo(init_pos4_L, 0, swapDuration)
    timeHelper.sleep(swapDuration)

    # First swap
    cf1.goTo(init_pos2_H, 0, swapDuration)
    cf2.goTo(init_pos1_L, 0, swapDuration)
    cf3.goTo(init_pos4_H, 0, swapDuration)
    cf4.goTo(init_pos3_L, 0, swapDuration)
    timeHelper.sleep(swapDuration)

    # Second initial positions
    cf1.goTo(init_pos2_L, 0, swapDuration)
    cf2.goTo(init_pos1_H, 0, swapDuration)
    cf3.goTo(init_pos4_L, 0, swapDuration)
    cf4.goTo(init_pos3_H, 0, swapDuration)
    timeHelper.sleep(swapDuration)

    # Second swap
    cf1.goTo(init_pos1_L, 0, swapDuration)
    cf2.goTo(init_pos2_H, 0, swapDuration)
    cf3.goTo(init_pos3_L, 0, swapDuration)
    cf4.goTo(init_pos4_H, 0, swapDuration)
    timeHelper.sleep(swapDuration)

    # Back to flat position
    cf1.goTo(pos1, 0, swapDuration)
    cf2.goTo(pos2, 0, swapDuration)
    cf3.goTo(pos3, 0, swapDuration)
    cf4.goTo(pos4, 0, swapDuration)
    timeHelper.sleep(swapDuration) 


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=object_height / 2, duration=3.0 + object_height)
    timeHelper.sleep(3.0 + object_height)

    height_gap = 0.3 
    swapDuration = 8  

    layer = 0  # Initialize layer counter



    while layer < num_layers and swarm.input.checkIfAnyButtonIsPressed() is None:
        if layer % 2 == 0:
            swapPosition_twice(allcfs.crazyflies[0], allcfs.crazyflies[1],allcfs.crazyflies[2], allcfs.crazyflies[3], height_gap, swapDuration)
        else:
            swapPosition_twice(allcfs.crazyflies[3], allcfs.crazyflies[0],allcfs.crazyflies[1], allcfs.crazyflies[2], height_gap, swapDuration)

        layer += 1  # Increment the layer counter

    allcfs.land(targetHeight=0.01, duration=5)
    timeHelper.sleep(5)
