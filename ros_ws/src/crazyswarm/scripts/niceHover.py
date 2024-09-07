
import numpy as np
from pycrazyswarm import *

INITIAL_HEIGHT = 1.5  # Initial takeoff height
FINAL_HEIGHT = 0.5    # Final height after lifting up
TAKEOFF_DURATION = 12.0
LIFT_DURATION = 10.0

if __name__ == "__main__":
    swarm = Crazyswarm("../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Take off to initial height
    allcfs.takeoff(targetHeight=INITIAL_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    # # Lift up slowly to the final height
    # for cf in allcfs.crazyflies:
    #     initial_pos = np.array(cf.initialPosition) + np.array([0, 0, INITIAL_HEIGHT])
    #     target_pos = initial_pos + np.array([0, 0, FINAL_HEIGHT])
    #     cf.goTo(target_pos, 0, LIFT_DURATION)
    # timeHelper.sleep(LIFT_DURATION)

    # Wait until any button is pressed to start landing
    while swarm.input.checkIfAnyButtonIsPressed() is None:
        timeHelper.sleep(0.1)
        print("waiting button")

    # Land
    allcfs.land(targetHeight=0.01, duration=10)
    timeHelper.sleep(10)