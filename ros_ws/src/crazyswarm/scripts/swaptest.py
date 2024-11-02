import time
import numpy as np
from pycrazyswarm import *
from class_circle_traj_2points import Collinear_to_traj
from class_catenary_trajectory import CatenaryTrajectory
import sys
import os

sys.path.append('/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/scripts/optitrack_natnet')

from optitrack_natnet.NatNetClient import NatNetClient
from optitrack_natnet.util import quaternion_to_euler

# Constants
INITIAL_HEIGHT = 0.7   # Hover height for drones
SQUARE_SIDE = 1.5      # Length of the side of the square around the object
LIFT_HEIGHT = 0.5      # Height difference for the lift-up behavior
SWAP_DURATION = 7.0    # Duration for drone swaps
LAND_DURATION = 7.0    # Duration for landing drones
currentLayer = 3       # Start with the third layer

clientAddress = "192.168.0.64"
optitrackServerAddress = "192.168.0.4"

# Global variables for object tracking
objects = {}
object_rotations = {}



def move_smoothly_simultaneously(swarm, allcfs, start_positions, end_positions, duration, timeHelper, object_pt):
    steps = int(duration * 100)  # 100 steps per second
    time_step = duration / steps  # Time for each step

    # Convert positions to NumPy arrays for calculations
    start_positions = [np.array(pos) for pos in start_positions]
    end_positions = [np.array(pos) for pos in end_positions]

    for step in range(steps):
        alpha = step / float(steps)
        for i, cf in enumerate(allcfs.crazyflies):
            # Interpolate current position smoothly
            current_pos = (1 - alpha) * start_positions[i] + alpha * end_positions[i]
            
            # Maintain altitude to avoid drops
            current_pos[2] = max(current_pos[2], INITIAL_HEIGHT)

            # Calculate yaw angle toward the object point
            yaw_angle = np.arctan2(object_pt[1] - current_pos[1], object_pt[0] - current_pos[0])

            # Move drone to current interpolated position
            cf.cmdPosition(current_pos, yaw=yaw_angle)
            
            # Check for emergency stop
            if swarm.input.checkIfAnyButtonIsPressed():
                print("Emergency landing initiated")
                allcfs.land(targetHeight=0.01, duration=10)
                timeHelper.sleep(10)
                for cf in allcfs.crazyflies:
                    cf.emergency()
                    timeHelper.sleep(1)
                return  # Exit if emergency stop is triggered

        timeHelper.sleep(time_step)  # Consistent timing for each step


def calculate_positions_around_object(num_drones, object_pt, radius):
    """
    Calculate positions around the object point in a circular formation.

    Args:
    num_drones: The number of drones.
    object_pt: The reference point (object) at the center of the circle.
    radius: The radius of the circle.

    Returns:
    List of target positions for each drone.
    """
    angles = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
    positions = []

    for angle in angles:
        x = object_pt[0] + radius * np.cos(angle)
        y = object_pt[1] + radius * np.sin(angle)
        z = object_pt[2] + INITIAL_HEIGHT
        positions.append([x, y, z])

    return positions

def assign_drones_to_nearest_angles(swarm, allcfs, object_pt, radius):
    """
    Assign each drone to the nearest available angle around the object point.

    Args:
    swarm: The CrazySwarm object.
    allcfs: The list of all Crazyflie drones.
    object_pt: The reference point (object) at the center of the circle.
    radius: The radius of the circle.

    Returns:
    List of target positions for each drone.
    """
    num_drones = len(allcfs.crazyflies)
    target_positions = calculate_positions_around_object(num_drones, object_pt, radius)
    start_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]

    assigned_positions = []
    used_indices = set()

    for start_pos in start_positions:
        min_distance = float('inf')
        nearest_index = -1

        for i, target_pos in enumerate(target_positions):
            if i in used_indices:
                continue

            distance = np.linalg.norm(start_pos - target_pos)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        assigned_positions.append(target_positions[nearest_index])
        used_indices.add(nearest_index)

    return assigned_positions

def move_to_circular_formation(swarm, allcfs, timeHelper, object_pt, radius):
    """
    Function to move drones to a circular formation around an object.

    Args:
    swarm: The CrazySwarm object.
    allcfs: The list of all Crazyflie drones.
    timeHelper: The TimeHelper object.
    object_pt: The reference point (object) at the center of the circle.
    radius: The radius of the circle.
    """
    target_positions = assign_drones_to_nearest_angles(swarm, allcfs, object_pt, radius)
    
    # Get the current start positions of all drones
    start_positions = [cf.position() for cf in allcfs.crazyflies]
    
    # Move smoothly to the assigned positions
    move_smoothly_simultaneously(swarm, allcfs, start_positions, target_positions, SWAP_DURATION, timeHelper, object_pt)
def swap_drones_counterclockwise(allcfs, drone1, drone2, timeHelper, duration, object_pt):
    # Get starting positions for all drones
    start_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]
    
    # Find the indices of the two drones in the allcfs.crazyflies list
    drone1_index = allcfs.crazyflies.index(drone1)
    drone2_index = allcfs.crazyflies.index(drone2)
    
    start_pos_1 = start_positions[drone1_index]
    start_pos_2 = start_positions[drone2_index]

    midpoint = (start_pos_1 + start_pos_2) / 2  # Calculate circle center point
    vec1 = start_pos_1 - midpoint
    vec2 = start_pos_2 - midpoint

    steps = int(duration * 100)  # Set timing steps for 100 Hz update frequency
    for step in range(steps):
        alpha = step / float(steps)
        theta = alpha * np.pi  # 180 degrees in radians

        # Calculate rotation matrix for smooth counterclockwise motion
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        
        # Rotate vectors to get new positions around the midpoint
        new_vec1 = np.dot(R, vec1[:2])
        new_vec2 = np.dot(R, vec2[:2])

        # Construct new positions while maintaining altitude
        new_pos_1 = np.append(midpoint[:2] + new_vec1, max(start_pos_1[2], INITIAL_HEIGHT))
        new_pos_2 = np.append(midpoint[:2] + new_vec2, max(start_pos_2[2], INITIAL_HEIGHT))

        # Command all drones, keeping non-swapped drones at their current positions
        for i, cf in enumerate(allcfs.crazyflies):
            if swarm.input.checkIfAnyButtonIsPressed():
                print("Emergency landing initiated")
                allcfs.land(targetHeight=0.01, duration=10)
                timeHelper.sleep(10)
                for cf in allcfs.crazyflies:
                    cf.emergency()
                    timeHelper.sleep(1)
                return  # Exit if emergency stop is triggered

            if i == drone1_index:
                cf.cmdPosition(new_pos_1)
            elif i == drone2_index:
                cf.cmdPosition(new_pos_2)
            else:
                # Hold position for non-swapping drones
                cf.cmdPosition(start_positions[i])

        timeHelper.sleep(0.01)  # Ensure timing consistency for each iteration


def receive_rigid_body_frame(id, position, rotation_quaternion):
    global objects, object_rotations
    objects[id] = position

    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    object_rotations[id] = (rotx, roty, rotz)

def setup_optitrack_client():
    client = NatNetClient()
    client.set_client_address(clientAddress)
    client.set_server_address(optitrackServerAddress)
    client.set_use_multicast(True)
    client.rigid_body_listener = receive_rigid_body_frame
    client.run()  # Start the client as part of the initial setup, assume it runs indefinitely

def get_object_position(id):
    # Returns the position and rotation if available
    position = objects.get(id)
    rotation = object_rotations.get(id)
    return position, rotation

if __name__ == "__main__":
    swarm = Crazyswarm("../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    target_object_id = 530
    setup_optitrack_client()
    allcfs = swarm.allcfs
    print(f"Command-line arguments: {sys.argv}")

    # Enable collision avoidance
    ellipsoid_radii = [0.15, 0.15, 0.15]

    for cf in allcfs.crazyflies:
        others = [other_cf for other_cf in allcfs.crazyflies if other_cf.id != cf.id]
        cf.enableCollisionAvoidance(others, ellipsoid_radii)
        print(f"Collision Avoidance Enabled for CF {cf.id} with ellipsoid radii {ellipsoid_radii}")

    # if '--sim' in sys.argv:s
    #     print("Running in simulation mode")
    #     position = np.array([0.0, 0.0, 0.0])  # Use [0.0, 0.0, 0.0] as object position in simulation
    # else:

    position, rotation = get_object_position(target_object_id)
    while position == None:
        position, rotation = get_object_position(target_object_id)
    if position:
        print(f"Position of object {target_object_id}:", position)
    else:
        print("no data from object")
    
    
    print(f"Target Object Position: {position}")
    
    # Take off to initial hover height
    allcfs.takeoff(targetHeight=INITIAL_HEIGHT, duration=5.0)
    timeHelper.sleep(5.0)

    # # Move drones into a circular formation around the object
    # move_to_circular_formation(swarm, allcfs, timeHelper, position, SQUARE_SIDE / 2)
    


    # Use a for loop to resolve hitch starting from currentLayer = 3 down to 1
    for layer in range(currentLayer, 0, -1):
        print(f"Resolving hitch for layer {layer}")
        
        if layer % 2 == 1:
            # Odd Layer: Swap Drone 1 ↔ Drone 2 and Drone 3 ↔ Drone 4
            print(f"Odd Layer {layer}: Swapping Drone 1 ↔ Drone 2 and Drone 3 ↔ Drone 4")
            swap_drones_counterclockwise(allcfs, allcfs.crazyflies[0], allcfs.crazyflies[1], timeHelper, SWAP_DURATION, position)
            swap_drones_counterclockwise(allcfs, allcfs.crazyflies[2], allcfs.crazyflies[3], timeHelper, SWAP_DURATION, position)
        else:
            # Even Layer: Swap Drone 1 ↔ Drone 3 and Drone 2 ↔ Drone 4
            print(f"Even Layer {layer}: Swapping Drone 1 ↔ Drone 3 and Drone 2 ↔ Drone 4")
            swap_drones_counterclockwise(allcfs, allcfs.crazyflies[0], allcfs.crazyflies[2], timeHelper, SWAP_DURATION, position)
            swap_drones_counterclockwise(allcfs, allcfs.crazyflies[1], allcfs.crazyflies[3], timeHelper, SWAP_DURATION, position)
    
    # Wait a bit after resolving all layers to observe results
    timeHelper.sleep(2.0)

    # Land all drones
    allcfs.land(targetHeight=0.01, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION)

    print("Landing completed.")