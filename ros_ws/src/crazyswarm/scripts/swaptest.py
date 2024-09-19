import numpy as np
from pycrazyswarm import *

# Constants
INITIAL_HEIGHT = 1.5  # Hover height for drones
SWAP_DURATION = 2.0   # Duration for drone swaps
LAND_DURATION = 10.0  # Duration for landing drones
currentLayer = 3      # Start with the third layer

def swap_drones_counterclockwise(drone1, drone2, timeHelper, duration):
    """
    Function to smoothly swap two drones along a counterclockwise circular path 
    over a set duration using interpolation.
    
    Args:
    drone1: The first Crazyflie drone object.
    drone2: The second Crazyflie drone object.
    timeHelper: Object to manage time for smooth movements.
    duration: The duration over which the swap will occur.
    
    Returns:
    None
    """
    
    # Get the start positions of the two drones
    start_pos_1 = np.array(drone1.position())
    start_pos_2 = np.array(drone2.position())
    
    # Calculate the midpoint between the two drones (center of the circle)
    midpoint = (start_pos_1 + start_pos_2) / 2
    
    # Calculate vectors from the midpoint to the drones
    vec1 = start_pos_1 - midpoint
    vec2 = start_pos_2 - midpoint
    
    # Number of steps for smooth movement (e.g., 100 steps per second)
    steps = int(duration * 100)
    
    # Interpolate the circular movement over the steps
    for step in range(steps):
        alpha = step / float(steps)
        # Rotation angle for both drones, rotating counterclockwise (theta increases)
        theta =2* alpha * np.pi  # 180 degrees in radians
        
        # Create the rotation matrix for counterclockwise rotation
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        
        # Apply the rotation to the vectors (2D rotation in the x-y plane)
        new_vec1 = np.dot(R, vec1[:2])  # Only rotate in x-y plane
        new_vec2 = np.dot(R, vec2[:2])
        
        # Compute the new positions by adding the rotated vectors to the midpoint
        new_pos_1 = midpoint[:2] + new_vec1
        new_pos_2 = midpoint[:2] + new_vec2
        
        # Retain the original z-coordinate (altitude)
        new_pos_1 = np.append(new_pos_1, start_pos_1[2])
        new_pos_2 = np.append(new_pos_2, start_pos_2[2])
        
        # Command the drones to move to their new positions
        drone1.cmdPosition(new_pos_1)
        drone2.cmdPosition(new_pos_2)
        
        # Wait a short amount of time to create a smooth transition
        timeHelper.sleep(0.01)  # 10 ms per step


if __name__ == "__main__":
    swarm = Crazyswarm("../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Take off to initial hover height
    allcfs.takeoff(targetHeight=INITIAL_HEIGHT, duration=5.0)
    timeHelper.sleep(5.0)

    # Use a for loop to resolve hitch starting from currentLayer = 3 down to 1
    for layer in range(currentLayer, 0, -1):
        print(f"Resolving hitch for layer {layer}")
        
        if layer % 2 == 1:
            # Odd Layer: Swap Drone 1 ↔ Drone 2 and Drone 3 ↔ Drone 4
            print(f"Odd Layer {layer}: Swapping Drone 1 ↔ Drone 2 and Drone 3 ↔ Drone 4")
            swap_drones_counterclockwise(allcfs.crazyflies[0], allcfs.crazyflies[1], timeHelper, SWAP_DURATION)
            swap_drones_counterclockwise(allcfs.crazyflies[2], allcfs.crazyflies[3], timeHelper, SWAP_DURATION)
        else:
            # Even Layer: Swap Drone 1 ↔ Drone 3 and Drone 2 ↔ Drone 4
            print(f"Even Layer {layer}: Swapping Drone 1 ↔ Drone 3 and Drone 2 ↔ Drone 4")
            swap_drones_counterclockwise(allcfs.crazyflies[0], allcfs.crazyflies[2], timeHelper, SWAP_DURATION)
            swap_drones_counterclockwise(allcfs.crazyflies[1], allcfs.crazyflies[3], timeHelper, SWAP_DURATION)
    
    # Wait a bit after resolving all layers to observe results
    timeHelper.sleep(2.0)

    # Land all drones
    allcfs.land(targetHeight=0.01, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION)

    print("Landing completed.")
