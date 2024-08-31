"""Takeoff-hover-land for all CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np
import rospy
from geometry_msgs.msg import Point



TAKEOFF_DURATION = 8
# HOVER_DURATION = 10.0



def moveToward(cf, x, y, z, duration):

    # Get the initial position of the Crazyflie
    initPos = np.array(cf.initialPosition)
    targetPos = initPos + np.array([x, y, z])
    cf.goTo(targetPos, yaw=0, duration=duration)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    allcfs = swarm.allcfs

    #takeoff
    for cf in allcfs.crazyflies:
        cf.takeoff(targetHeight=1.5, duration=TAKEOFF_DURATION)

    timeHelper.sleep(TAKEOFF_DURATION + 3)



    #move toward
    for cf in allcfs.crazyflies:
        moveToward(cf, x=1., y=0., z=0.0, duration=10)
    timeHelper.sleep(5)

    # Land all Crazyflies
    for cf in allcfs.crazyflies:
        cf.land(targetHeight=0.04, duration=3)

    timeHelper.sleep(3) 

if __name__ == "__main__":
    main()
