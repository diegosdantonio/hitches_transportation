"""Takeoff-hover-land for all CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 12
HOVER_DURATION = 10.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    allcfs = swarm.allcfs
    for cf in allcfs.crazyflies:
        cf.takeoff(targetHeight= 2.5, duration=TAKEOFF_DURATION)

    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    for cf in allcfs.crazyflies:
        cf.land(targetHeight=0.04, duration = 8)

    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
