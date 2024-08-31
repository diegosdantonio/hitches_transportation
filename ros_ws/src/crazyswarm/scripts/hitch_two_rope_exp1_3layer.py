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

clientAddress = "192.168.0.24" #"192.168.0.59"
optitrackServerAddress = "192.168.0.4"

objects = {}
object_rotations = {}






global object_pt

desireLayer = 3

ropeLength = 3.57
Rotation_num = 1
ellipsoid_radii = [0.12, 0.12, 0.12]
duration_per_layer = 45  # Duration for each layer in seconds
initial_position_duration = 10  # Duration to move to initial positions

'''
    movement function:
            1. move_smoothly_simultaneously to to move multiple robot
        at once in given start_positions and end_positions

            2. move_circularly to move multiple robot at once in circular
        motion with center in object point

'''
def move_smoothly_simultaneously(swarm, allcfs, start_positions, end_positions, duration, timeHelper, object_pt):
    steps = int(duration * 100)  # 100 steps per second
    for step in range(steps):
        alpha = step / float(steps)
        for i, cf in enumerate(allcfs.crazyflies):
            current_pos = (1 - alpha) * start_positions[i] + alpha * end_positions[i]
            yaw_angle = np.arctan2(object_pt[1] - current_pos[1], object_pt[0] - current_pos[0])
            cf.cmdPosition(current_pos, yaw=yaw_angle)  # Make sure the angle is in radians

            if swarm.input.checkIfAnyButtonIsPressed():
                print("Emergency landing initiated 1")
                allcfs.land(targetHeight=0.01, duration=10)
                timeHelper.sleep(10)
                for cf in allcfs.crazyflies:
                    cf.emergency()  # Assuming each Crazyflie object has a cmdStop method
                    timeHelper.sleep(1)
                break
        timeHelper.sleep(0.005)


def move_circularly(swarm, allcfs, start_positions, end_positions, duration, timeHelper, center_point):
    steps = int(duration * 100)
    start_radii = [np.linalg.norm(start_pos - center_point) for start_pos in start_positions]  # Radii from center to start positions
    end_radii = [np.linalg.norm(end_pos - center_point) for end_pos in end_positions]  # Radii from center to end positions if you want to change radius
    start_angles = [np.arctan2(start_pos[1] - center_point[1], start_pos[0] - center_point[0]) for start_pos in start_positions]
    end_angles = [np.arctan2(end_pos[1] - center_point[1], end_pos[0] - center_point[0]) for end_pos in end_positions]

    # Calculate angle differences ensuring shortest rotation direction
    angle_differences = [((end_angle - start_angle + np.pi) % (2 * np.pi) - np.pi) for start_angle, end_angle in zip(start_angles, end_angles)]

    for step in range(steps):
        alpha = step / float(steps)
        for i, cf in enumerate(allcfs.crazyflies):
            radius = start_radii[i] + alpha * (end_radii[i] - start_radii[i])  # Interpolating the radius
            angle = start_angles[i] + alpha * angle_differences[i]
            x = center_point[0] + radius * np.cos(angle)
            y = center_point[1] + radius * np.sin(angle)
            z = (1 - alpha) * start_positions[i][2] + alpha * end_positions[i][2]  # Linear interpolation for z
            current_pos = np.array([x, y, z])
            yaw_angle = np.arctan2(center_point[1] - y, center_point[0] - x)

            cf.cmdPosition(current_pos, yaw=yaw_angle)

            if swarm.input.checkIfAnyButtonIsPressed():
                print("Emergency landing initiated 2")
                allcfs.land(targetHeight=0.01, duration=10)
                timeHelper.sleep(10)
                for cf in allcfs.crazyflies:
                    cf.emergency()  # Assuming each Crazyflie object has a cmdStop method
                    timeHelper.sleep(1)
                break
        timeHelper.sleep(0.005) 

'''
    natnet to get object position from optitrack
'''

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


def move_away_from_object(swarm, allcfs, current_positions, object_pt, displacement_distance, timeHelper):

    displacement_vectors = [
        (pos - object_pt) / np.linalg.norm(pos - object_pt) * displacement_distance
        for pos in current_positions
    ]
    away_positions = [
        current_positions[i] + displacement_vectors[i] 
        for i in range(len(current_positions))
    ]
    
    move_smoothly_simultaneously(swarm, allcfs, current_positions, away_positions, 3, timeHelper, object_pt)
    return away_positions  # Return the new positions after moving away.


def calculate_rope_length(layer, allcfs, object_pt, radius):
    """
    Calculate the rope lengths for Crazyflies based on their current layer and positions around an object point.

    :param layer: Current layer number to determine which contacts to use.
    :param allcfs: Reference to all Crazyflies.
    :param object_pt: Numpy array of the object's center position.
    :param radius: Radius to define the contact points around the object point.
    :return: The calculated rope length for the current configuration.
    """
    # Retrieve current positions of all Crazyflies
    current_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]

    # Define contact points around the object
    contacts = {
        'contact1': object_pt + np.array([0, -radius, 0]),
        'contact2': object_pt + np.array([0, radius, 0]),
        'contact3': object_pt + np.array([-radius, 0, 0]),
        'contact4': object_pt + np.array([radius, 0, 0])
    }

    # Calculate distances from each robot to its designated contact point
    distances = {
        'dist_robot1_contact1': np.linalg.norm(current_positions[0] - contacts['contact1']),
        'dist_robot1_contact2': np.linalg.norm(current_positions[0] - contacts['contact2']),
        'dist_robot2_contact1': np.linalg.norm(current_positions[1] - contacts['contact1']),
        'dist_robot2_contact2': np.linalg.norm(current_positions[1] - contacts['contact2']),
        'dist_robot3_contact3': np.linalg.norm(current_positions[2] - contacts['contact3']),
        'dist_robot3_contact4': np.linalg.norm(current_positions[2] - contacts['contact4']),
        'dist_robot4_contact3': np.linalg.norm(current_positions[3] - contacts['contact3']),
        'dist_robot4_contact4': np.linalg.norm(current_positions[3] - contacts['contact4'])
    }

    # Calculate rope length based on layer
    if layer % 2 == 1:
        # Odd layer: Use distances for robots 1 and 2
        ropelength = ((distances['dist_robot1_contact1'] + distances['dist_robot1_contact2']) +
                      (distances['dist_robot2_contact1'] + distances['dist_robot2_contact2'])) / 2
    else:
        # Even layer: Use distances for robots 3 and 4
        ropelength = ((distances['dist_robot3_contact3'] + distances['dist_robot3_contact4']) +
                      (distances['dist_robot4_contact3'] + distances['dist_robot4_contact4'])) / 2

    return ropelength



def square_and_lift(swarm, allcfs, current_positions, object_pt, initial_radius, max_lift_height, duration, timeHelper):
    steps = int(duration * 100)  # Example: 100 steps per second for the given duration
    lift_per_step = max_lift_height / steps
    radius_reduction_per_step = initial_radius / steps  # Linear reduction of radius

    for step in range(steps):
        current_lift_height = lift_per_step * step
        current_radius = initial_radius - radius_reduction_per_step * step

        # Calculate new positions for this step
        step_positions = []
        for pos in current_positions:
            # Calculate current rope length if needed, else use decreasing radius
            rope_length = np.linalg.norm(pos - object_pt)  # Or use current_radius directly

            # Determine quadrant and adjust position
            dx = pos[0] - object_pt[0]
            dy = pos[1] - object_pt[1]
            if dx >= 0 and dy >= 0:
                angle = np.pi / 4
            elif dx < 0 and dy >= 0:
                angle = 3 * np.pi / 4
            elif dx < 0 and dy < 0:
                angle = 5 * np.pi / 4
            else:
                angle = 7 * np.pi / 4

            new_x = object_pt[0] + current_radius * np.cos(angle)
            new_y = object_pt[1] + current_radius * np.sin(angle)
            new_z = pos[2] + current_lift_height

            step_positions.append(np.array([new_x, new_y, new_z]))

        # Move the Crazyflies to the new positions for this step
        move_smoothly_simultaneously(swarm, allcfs, current_positions, step_positions, 1, timeHelper, object_pt)  # Assume very quick, dynamic updates
        current_positions = step_positions  # Update current positions for next step

    allcfs.land(targetHeight=0.01, duration=10)
    timeHelper.sleep(10)



def main():
    setup_optitrack_client()  # Start the client to begin receiving data


    '''
        Variable initializati0
    '''
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    currentRopeLength = ropeLength
    currentLayer = 1

    target_object_id = 530  # ID of the object you want to track
    hight_decrease = 0.05 # Value ecrease after first layer
    normal_shift = 0.42 # Value shifter for slack
    shrink_distance = 0.000  # distance shrink during the rotation for cicular trajctory within layers
    object_r = 0.15
    flag = 0



    '''
        1. Getting position of the object and deine hitch point
        2. Enable Collision Avoidance an take off
    '''
    position, rotation = get_object_position(target_object_id)
    if position and rotation:
        print(f"Position of object {target_object_id}:", position, "Rotation:", rotation)
        object_pt = position + np.array([0.0,0.0,normal_shift])
    else:
        print("no data from object")
        return
   

    print(f"target height = {object_pt[2]}")

        # Define two points in 3D space
    pt1 = np.array([0.22 , 0.0, 0.0]) + object_pt
    pt2 = np.array([-0.22, 0.0,0.0]) + object_pt

    pt3 = np.array([0.0, 0.22,0.0]) + object_pt
    pt4 = np.array([0.0, -0.22,0.0]) + object_pt


    start1 = object_pt + np.array([ropeLength/2 , -object_r * 4, 0.0]) 
    start2 = object_pt + np.array([ropeLength/2, object_r * 4, 0.0]) 
    start3 = object_pt + np.array([-ropeLength/2 , -object_r * 4, 0.0]) 
    start4 = object_pt + np.array([-ropeLength/2, object_r * 4, 0.0]) 





    for cf in allcfs.crazyflies:
        # List of other Crazyflies for the current Crazyflie to avoid
        others = [other_cf for other_cf in allcfs.crazyflies if other_cf.id != cf.id]
        cf.enableCollisionAvoidance(others, ellipsoid_radii)
        print(f"Collision Avoidance Enabled for CF {cf.id} with ellipsoid radii {ellipsoid_radii}")

    for cf in allcfs.crazyflies:
        print("Takeoff")
        cf.takeoff(targetHeight=object_pt[2] , duration=5)
    timeHelper.sleep(5)  # Ensure that takeoff completes

    start_time = timeHelper.time()



    '''
        1. big while loop to each layer
        2. calculate initial position for four robot
    '''    
    while currentLayer <= desireLayer and swarm.input.checkIfAnyButtonIsPressed() == None:
        # Create Collinear_to_traj object and calculate initial positions
        # if currentLayer == 1:
        #         pt1 = pt1 + np.array([0.0,0.0,slack_shift])
        #         pt2 = pt2 + np.array([0.0,0.0,slack_shift])

        #         pt3 = pt3 + np.array([0.0,0.0,slack_shift])
        #         pt4 = pt4 + np.array([0.0,0.0,slack_shift])
        CT = Collinear_to_traj([pt1, pt2, pt3, pt4], object_pt, object_r, currentRopeLength, currentLayer)
        
        s1, s2, s3, s4 = CT.calculate_rope_distances(currentLayer, currentRopeLength) 

        



        if currentLayer % 2 == 1:
            
            init1, init2, init3, init4 = CT.calculate_four_robot_position(pt1, pt2, s1, s2, s3, s4)
        else:
            init1, init2, init3, init4 = CT.calculate_four_robot_position(pt3, pt4, s1, s2, s3, s4)
        
        # if currentLayer == 1:
        #         pt1 = pt1 - np.array([0,0,slack_shift])
        #         pt2 = pt2 - np.array([0,0,slack_shift])

        #         pt3 = pt3 - np.array([0,0,slack_shift])
        #         pt4 = pt4 - np.array([0,0,slack_shift])
        #         currentRopeLength = 3.5


        # Move Crazyflies to initial positions smoothly and simultaneously
        start_positions = [cf.position() for cf in allcfs.crazyflies]
        end_positions = [init1, init2, init3, init4]
        if currentLayer == 1:
            end_positions = [ start1, start2, start3, start4]
        move_circularly(swarm, allcfs, start_positions, end_positions, initial_position_duration, timeHelper, object_pt)
        start_positions = end_positions  # Update start positions after reaching initial positions
        print("reach initial position")

        layer_start_time = timeHelper.time()


        # Perform the trajectory for the current layer
        init1_rep = init1
        init2_rep = init2
        init3_rep = init3
        init4_rep = init4

        print("start1:", start1)
        print("start2:", start2)
        print("start3:", start3)
        print("start4:", start4)


        print("init1:", init1)
        print("init2:", init2)
        print("init3:", init3)
        print("init4:", init4)

        while True:
            current_time = timeHelper.time() - layer_start_time
            if current_time >= duration_per_layer:
                t = Rotation_num  # Normalize to ensure it reaches exactly the intended rotation number
            else:
                t = (current_time / duration_per_layer) * Rotation_num


            if currentLayer == 1 and current_time < duration_per_layer/2:
                if flag == 0:
                    print("L1H1")
                    flag = 1
                r1_pos, dump = CT.trajz_to_t(t, start1, init2, 1)
                r2_pos, dump = CT.trajz_to_t(t, start2, init1, -1)
                r3_pos, dump = CT.trajz_to_t(t, start3, init4, -1)
                r4_pos, dump = CT.trajz_to_t(t, start4, init3, 1)
            elif currentLayer == 1 and current_time >= duration_per_layer/2:
                if flag == 1:
                    print("L1H2")
                    flag = 2
                dump, r2_pos = CT.trajz_to_t(t, init1, init2, 1)
                dump, r1_pos = CT.trajz_to_t(t, init2, init1, -1)
                dump, r4_pos = CT.trajz_to_t(t, init3, init4, -1)
                dump, r3_pos = CT.trajz_to_t(t, init4, init3, 1)


            elif currentLayer % 2 == 1:
                r1_pos, r2_pos = CT.trajz_to_t(t, init1_rep, init2_rep, 1)
                r3_pos, r4_pos = CT.trajz_to_t(t, init3_rep, init4_rep, -1)
                unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
                unit_vector_init2_to_pt1 = (pt1 - init2) / np.linalg.norm(pt1 - init2)
                unit_vector_init3_to_pt2 = (pt2 - init3) / np.linalg.norm(pt2 - init3)
                unit_vector_init4_to_pt2 = (pt2 - init4) / np.linalg.norm(pt2 - init4)

                init1_rep = init1_rep + unit_vector_init1_to_pt1 * shrink_distance
                init2_rep = init2_rep + unit_vector_init2_to_pt1 * shrink_distance
                init3_rep  = init3_rep + unit_vector_init3_to_pt2 * shrink_distance
                init4_rep  = init4_rep + unit_vector_init4_to_pt2 * shrink_distance
            else:
                r1_pos, r3_pos = CT.trajz_to_t(t, init1_rep, init3_rep, 1)
                r2_pos, r4_pos = CT.trajz_to_t(t, init2_rep, init4_rep, -1)
                unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
                unit_vector_init3_to_pt1 = (pt1 - init3) / np.linalg.norm(pt1 - init3)
                unit_vector_init2_to_pt2 = (pt2 - init2) / np.linalg.norm(pt2 - init2)
                unit_vector_init4_to_pt2 = (pt2 - init4) / np.linalg.norm(pt2 - init4)
                
                init1_rep  = init1_rep + unit_vector_init1_to_pt1 * shrink_distance
                init2_rep  = init2_rep + unit_vector_init2_to_pt2 * shrink_distance
                init3_rep  = init3_rep + unit_vector_init3_to_pt1 * shrink_distance
                init4_rep  = init4_rep + unit_vector_init4_to_pt2 * shrink_distance

            new_positions = [r1_pos, r2_pos, r3_pos, r4_pos]
            move_smoothly_simultaneously(swarm, allcfs, start_positions, new_positions, 1, timeHelper, object_pt)
            start_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]  # Update start positions for next move

            if current_time >= duration_per_layer:
                print("Completed full rotation for the layer")
                current_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]
                
                print("moveaway")
                i = move_away_from_object(swarm, allcfs, current_positions, object_pt, 0.05, timeHelper)
                break

            #decrease init


            unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
            unit_vector_init2_to_pt1 = (pt1 - init2) / np.linalg.norm(pt1 - init2)

            

        '''
            increase tension and move lift up the object
        '''




        if currentLayer == desireLayer:
            print("Adjusting positions to maintain rope length while forming a square.")

            # Retrieve current positions of all Crazyflies
            current_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]

            # Compute new positions based on maintaining rope length and moving to a square formation
            end_positions = []
            converge_distance = 0.65   # Total distance to move towards the center during lift
            for pos in current_positions:
                # Calculate current rope length (distance to object_pt)
                rope_length = np.linalg.norm(pos - object_pt)

                # Determine quadrant and calculate the corresponding angle for the square formation
                dx = pos[0] - object_pt[0]
                dy = pos[1] - object_pt[1]
                if dx >= 0 and dy >= 0:  # First quadrant
                    angle = np.pi / 4
                elif dx < 0 and dy >= 0:  # Second quadrant
                    angle = 3 * np.pi / 4
                elif dx < 0 and dy < 0:  # Third quadrant
                    angle = 5 * np.pi / 4
                else:  # Fourth quadrant
                    angle = 7 * np.pi / 4

                # Calculate the new position while maintaining the rope length
                new_x = object_pt[0] + rope_length * np.cos(angle)
                new_y = object_pt[1] + rope_length * np.sin(angle)
                new_z = pos[2]  # Maintain current altitude
                end_positions.append(np.array([new_x, new_y, new_z]))

            # Move the Crazyflies to their new positions
            move_smoothly_simultaneously(swarm, allcfs, current_positions, end_positions, 10, timeHelper, object_pt)

            liftupHeight = 1.2  # Set desired lift up height if lifting after reaching positions
            lifted_positions = []
            for end_pos in end_positions:
                # Gradually decrease the distance to the center as they lift up
                # Adjust object_pt to match dimensions by considering only x and y coordinates
                direction_vector = (object_pt[:2] - end_pos[:2])  # Direction from end position to center
                normalized_vector = direction_vector / np.linalg.norm(direction_vector) if np.linalg.norm(direction_vector) != 0 else np.array([0, 0])

                # Calculate final lifted position with convergence
                final_x = end_pos[0] + normalized_vector[0] * converge_distance
                final_y = end_pos[1] + normalized_vector[1] * converge_distance
                final_z = end_pos[2] + liftupHeight
                lifted_positions.append(np.array([final_x, final_y, final_z]))

            # Move to lifted positions
            move_smoothly_simultaneously(swarm, allcfs, end_positions, lifted_positions, 20, timeHelper, object_pt)
            # Landing
            allcfs.land(targetHeight=0.01, duration=10)
            timeHelper.sleep(10)

        # Move to the next layer   
        # 
        if currentLayer == 1:
            object_pt = object_pt - np.array([0.0, 0.0, hight_decrease])
            pt1 = pt1 - np.array([0.0,0.0,hight_decrease])
            pt2 = pt2 - np.array([0.0,0.0,hight_decrease]) 
            pt3 = pt3 - np.array([0.0,0.0,hight_decrease]) 
            pt4 = pt4 - np.array([0.0,0.0,hight_decrease])


        currentLayer += 1
        currentRopeLength = calculate_rope_length(currentLayer, allcfs, object_pt, object_r) + 0.05

        print(f"current ropelength: {currentRopeLength}")

    print("End of trajectory reached, preparing to land.")
    allcfs.land(targetHeight=0.01, duration=10)
    timeHelper.sleep(10)
    print("Landing complete.")

if __name__ == "__main__":
    main()
