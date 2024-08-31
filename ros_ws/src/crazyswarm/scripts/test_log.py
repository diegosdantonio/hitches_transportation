import rospy
from pycrazyswarm import *
from crazyswarm.msg import GenericLogData
import numpy as np
from threading import Thread

acc_l = None
acc_r = None
counter = 0

# Callback function for ROS subscriber
def ros_accel_callback_L(data):
    global acc_l
    acc_l = data.values

def ros_accel_callback_R(data):
    global acc_r
    acc_r = data.values

# Callback function for Crazyswarm log configuration

def main():
    global counter
    while counter < 400:
        counter += 1
        print(acc_l, acc_r)
        rospy.sleep(0.1)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('accel_subscriber', anonymous=True)

    # Subscribe to ROS topic for acceleration
    rospy.Subscriber('/cf5/accs', GenericLogData, ros_accel_callback_L)
    rospy.Subscriber('/cf6/accs', GenericLogData, ros_accel_callback_R)

    main()