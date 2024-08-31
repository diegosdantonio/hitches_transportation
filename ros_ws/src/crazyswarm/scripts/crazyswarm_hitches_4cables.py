#!/usr/bin/env python
import time

import numpy as np
from pycrazyswarm import *
from class_hitches import Quasi_satic_class
from class_circle_trajz import circle_traj

Z = 0.8


def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log


if __name__ == "__main__":

    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition:
      - 1.4
      - -1.
      - 0.
      type: medium
    - channel: 100
      id: 2
      initialPosition:
      - 1
      - -1.4
      - 0.
      type: medium
    - channel: 100
      id: 3
      initialPosition:
      - -1
      - -1.4
      - 0.
      type: medium
    - channel: 100
      id: 4
      initialPosition:
      - -1.4
      - -1.
      - 0.
      type: medium
    - channel: 100
      id: 5
      initialPosition:
      - -1.4
      - 1.
      - 0.
      type: medium
    - channel: 100
      id: 6
      initialPosition:
      - -1.
      - 1.4
      - 0.
      type: medium
    - channel: 100
      id: 7
      initialPosition:
      - 1.
      - 1.4
      - 0.
      type: medium
    - channel: 100
      id: 8
      initialPosition:
      - 1.4
      - 1.
      - 0.
      type: medium
    """

    # swarm = Crazyswarm()
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    TIMESCALE = 2

    allcfs.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(1.5+Z)

    p = 0.8
    p1 = [p, -p, 0.0]
    p2 = [-p, -p, 0.0]
    p3 = [-p, p, 0.0]
    p4 = [p, p, 0.0]
    raux = 0.25

    xd = np.array([p1, p2, p3, p4])

    xd_theta = np.array([np.deg2rad(-45)
                         , np.deg2rad(45)
                         , np.deg2rad(135)
                         , np.deg2rad(225)])

    r = np.array([raux, raux, raux, raux])

    C1 = circle_traj(xd, xd_theta, r)

    t_step =  0.08
    t = 0.01

    C1.update(0)
    posaux = C1.all

    for i, cf in enumerate(allcfs.crazyflies):
        pos = np.array(posaux[i]) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 2)

    timeHelper.sleep(0.1)

    swarm.input.waitUntilButtonPressed()
    ban = False

    while t < 2*np.pi and not swarm.input.checkIfButtonIsPressed():
        C1.update(t)
        posaux = C1.all

        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(posaux[i]) + np.array([0, 0, Z])
            cf.goTo(pos, 0, 2)

        timeHelper.sleep(0.1)
        t += t_step

    swarm.input.waitUntilButtonPressed()
    while t >= 0 and not swarm.input.checkIfButtonIsPressed():
        C1.update(t)
        posaux = C1.all

        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(posaux[i]) + np.array([0, 0, Z])
            cf.goTo(pos, 0, 2)

        timeHelper.sleep(0.1)
        t -= t_step

    swarm.input.waitUntilButtonPressed()
    print("press button to continue...")

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
