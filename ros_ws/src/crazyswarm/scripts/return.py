
import numpy as np
from pycrazyswarm import *

INITIAL_HEIGHT = 1.8  # Initial takeoff height
TAKEOFF_DURATION = 5.0
LIFT_DURATION = 6.0

if __name__ == "__main__":
    swarm = Crazyswarm("../launch/rodcrazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Take off to initial height
    allcfs.takeoff(targetHeight=INITIAL_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    # Lift up slowly to the final height
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, INITIAL_HEIGHT])
        cf.goTo(pos, 0, LIFT_DURATION)
    timeHelper.sleep(LIFT_DURATION)

    # Land
    allcfs.land(targetHeight=0.01, duration=3.0)
    timeHelper.sleep(5.0)
    allcfs.land(targetHeight=0.01, duration=3.0)
    timeHelper.sleep(5.0)