
import time
import sys
import os

# Path to the directory containing your modules
sys.path.append('/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/scripts/optitrack_natnet')

from optitrack_natnet.NatNetClient import NatNetClient
from optitrack_natnet.util import quaternion_to_euler

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)



if __name__ == "__main__":
    clientAddress = "192.168.0.39" #"192.168.0.59"
    optitrackServerAddress = "192.168.0.4"
    robot_ids = [480] #[3, 4]

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    while is_running:
        for robot_id in robot_ids:
            if robot_id in positions:
                # last position
                print(robot_id, 'Last position', positions[robot_id], ' rotation', rotations[robot_id])

        time.sleep(.1)
