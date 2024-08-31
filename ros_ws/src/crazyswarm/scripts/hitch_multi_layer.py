import time
import numpy as np
from pycrazyswarm import *
from class_hitch_traj import hitch_traj
from class_robot_control import robot_control

# Define multiple points and into a 2d array and each layer is in a subarray
points = np.array([
    [[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0], [-0.5, -0.5, 1.0], [0.5, -0.5, 1.0]],
    [[0.71, 0.0, 1.0], [0.0, 0.71, 1.0], [-0.71, 0.0, 1.0], [0.0, -0.71, 1.0]]
])


#robot and layer param, assume each points need 2
num_robot = len(points[0]) * 2
print("Number of required robots: ", num_robot)



# Parameters for the trajectory
object_pt = np.array([0.0, 0.0, 1.0])  #object xy and object height
object_radius = 0.15
ropeLength = 4.4
Rotation_num = 1
ellipsoid_radii = [0.15, 0.15,0.15] 

#process params
object_height = object_pt[2]

def ease_in_out(t):
    if t < 0.5:
        return 2 * t * t
    else:
        return -1 + (4 - 2 * t) * t


def calculate_distances(points):
    # Initialize lists to store distances for each set of points
    distances_set1 = []
    distances_set2 = []

    # Calculate distances for the first set of points
    for i in range(len(points[0]) - 1):  # Exclude the last index for standard looping
        point1 = points[0][i]
        point2 = points[0][i + 1]
        distance = np.linalg.norm(np.array(point1) - np.array(point2))
        distances_set1.append(distance)
    
    # Calculate distances for the second set of points
    for i in range(len(points[1]) - 1):  # Exclude the last index for standard looping
        point1 = points[1][i]
        point2 = points[1][i + 1]
        distance = np.linalg.norm(np.array(point1) - np.array(point2))
        distances_set2.append(distance)

    # Connect the last point to the first point to complete the loop for each set
    distance = np.linalg.norm(np.array(points[0][-1]) - np.array(points[0][0]))
    distances_set1.append(distance)

    distance = np.linalg.norm(np.array(points[1][-1]) - np.array(points[1][0]))
    distances_set2.append(distance)

    return distances_set1, distances_set2

def move_all_robots_simultaneously(timeHelper, robotControllers, positions, total_duration):
    desired_fps = 30  # Frames per second, adjust for smoother or coarser movements
    steps = int(total_duration * desired_fps)  # Total number of steps based on frames per second
    step_duration = total_duration / steps  # Duration each step should last

    for step in range(steps):
        t = step / float(steps)
        alpha = ease_in_out(t)  # Using ease-in-out for smoother start and end
        for robot_id, end_pos in positions.items():
            robot = next((rc for rc in robotControllers if rc.cf.id == robot_id), None)
            if robot:
                start_pos = robot.cf.position()
                current_pos = (1 - alpha) * np.array(start_pos) + alpha * np.array(end_pos)
                robot.cf.cmdPosition(current_pos.tolist(), yaw=0)
        timeHelper.sleep(step_duration)

    # Ensure all robots are exactly at their target positions at the end
    for robot_id, end_pos in positions.items():
        robot = next((rc for rc in robotControllers if rc.cf.id == robot_id), None)
        if robot:
            robot.cf.cmdPosition(end_pos.tolist(), yaw=0)

def main():


    #initialization
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    crazyflies = allcfs.crazyflies

    #mainparams
    desiredLayer = 2
    currentLayer = 0
    currentRopeLength = ropeLength
    liftupHeight = 1
    desired_fps = 30  # Desired frames per second for smooth motion
    duration_per_layer = 30  # Duration for each layer's movement in seconds

    steps_per_layer = duration_per_layer * desired_fps  # Total steps required for the layer
    duration_per_step = 1 / desired_fps  # Time duration for each step





    #check robots
    if len(crazyflies) != num_robot:
        print(f"Error: Number of Crazyflies ({len(crazyflies)}) does not match the required number ({num_robot}).")
        return  # Exit if the numbers don't match

    # Initialize robot controllers with unique identifiers
    robotControllers = [robot_control(cf, timeHelper, [0.15, 0.15, 0.15], cf.id) for cf in crazyflies]

    
    for robot in robotControllers:
        robot.enable_collision_avoidance(crazyflies)  # Pass the correct list

    # Take off each Crazyflie
    for robot in robotControllers:
        robot.takeoff(object_height, 5)
    timeHelper.sleep(5)

    #impove me 
    s1, s2 = calculate_distances(points)

    # try:
    while currentLayer <= desiredLayer:
        robot_index = 0
        if currentLayer % 2 == 0:
            robot_index = robot_index + 0
        else:
            robot_index = robot_index - 1

        s_length = (currentRopeLength - s1[0]) / 2
        print("current rope length: ", s_length)

        if currentLayer % 2 == 0:
            current_points = points[0]
        else:
            current_points = points[1]

        positions = {}  # Dictionary to store end positions with robot IDs

        # Dictionary to store robot pairs for trajectory calculation
        robot_pairs = {}

        # Loop to cover all points in pairs
        for i in range(len(current_points)):
            pt = current_points[i]
            next_pt = current_points[(i + 1) % len(current_points)]  # Wrap around using modulo
            last_pt = current_points[i - 1] if i > 0 else current_points[-1]


            HT = hitch_traj(pt, next_pt, last_pt, s_length, s_length)
            r1, r2 = HT.get_robot_position()


            k1 = robot_index
            k2 = robot_index + 1

            current_robot = robotControllers[k1]
            next_robot = robotControllers[k2]
            print(f"r1: {k1} (ID: {current_robot.flight_id}), r2: {k2} (ID: {next_robot.flight_id})")

            positions[current_robot.flight_id] = r1
            positions[next_robot.flight_id] = r2

            robot_index += 2  # Increment for the next pair

        # Move all robots simultaneously
        move_all_robots_simultaneously(timeHelper, robotControllers, positions, 5)
        print(f"move to initial position for layer: {currentLayer}")


        # Real-time trajectory movement
        layer_start_time = timeHelper.time()



        robot_index = 0
        if currentLayer % 2 == 0:
            robot_index = robot_index + 0
        else:
            robot_index = robot_index - 1

        print(f"robot index: {robot_index}")

        # Recalculate positions and update robot pairs
        for i in range(0, len(current_points)):  # Assume even number of points

            pt = current_points[i]
            # Hypothetically fetching robot controllers again; adjust logic as needed
            robot1 = robotControllers[robot_index]
            robot2 = robotControllers[robot_index + 1]

            HT = hitch_traj(pt, next_pt, pt, (currentRopeLength - s1[0]) / 2, (currentRopeLength - s1[0]) / 2)
            r1 = robot1.cf.position()
            r2 = robot2.cf.position()
            robot_pairs[(robot1, robot2)] = (r1, r2)
            robot_index += 2  # Increment by 2 for the next pair



        duration_per_step = 0.2  # Duration per update, consider this as a frame rate control

        while True:
            current_time = timeHelper.time() - layer_start_time
            t = current_time / duration_per_layer  # Normalize t to [0,1] interval
            print("t: ", t)

            # Calculate and update positions for each robot pair
            ind = 0  # Reset index for current_points
            for (robot1, robot2), (r1, r2) in robot_pairs.items():
                if ind >= len(current_points):
                    print("Index exceeds the length of current_points")
                    break
                
                r1_rotated, r2_rotated = HT.trajz_to_t(t, current_points[ind], r1, r2)


                positions[robot1.flight_id] = r1_rotated
                positions[robot2.flight_id] = r2_rotated
                ind += 1  # Move to the next point in current_points
            if t > 1:
                break  


            # Move robots based on newly calculated positions
            move_all_robots_simultaneously(timeHelper, robotControllers, positions, duration_per_step)
            timeHelper.sleep(duration_per_step)  # Control update frequency

        timeHelper.sleep(4)  # Wait some time before the next layer cycle
        currentRopeLength = currentRopeLength - (np.pi * object_radius) / (len(robotControllers) / 2)
        currentLayer += 1


    # except Exception as e:
    #     print(f"Interrupt detected: {str(e)}")    
    #     for robot in robotControllers:
    #         robot.emergency_land()
    #     timeHelper.sleep(5)
    #     print("Emergency landing completed for all robots.")



if __name__ == "__main__":
    main()