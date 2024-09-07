import rospy
from pycrazyswarm import *
from crazyswarm.msg import GenericLogData
import numpy as np
from threading import Thread
import matplotlib.pyplot as plt
import pickle

# Whether we would like to test the bendable force estimator
INIT_HEIGHT = 1.0
force_estimation = True
pos_errors_L = []
pos_errors_R = []
des_pos_L = []
des_pos_R = []
actual_pos_L = []
actual_pos_R = []
force_estimation_error_L = []
force_estimation_error_R = []
force_gt_L = []
force_gt_R = []
rate = 100
DT = 1.0/rate
bendable_feature = True
MAG_FEATURE = 1
MASS_MULTIPLIER = 8 # ra: 6

def increment_number_in_file(filename, clear=False):
    try:
        # Read the number from the file
        with open(filename, 'r') as file:
            number = int(file.read().strip())
    except FileNotFoundError:
        print(f"File {filename} not found. Creating the file with initial number 0.")
        number = 0

    # Increment the number
    if clear:
        number = 0
    else:
        number += 1
    # Write the incremented number back to the file
    with open(filename, 'w') as file:
        file.write(str(number))
    
    return number

REUSE_WEIGHT = False
file_identifier = "PID" if not force_estimation else "RLS_{}".format("ralpha" if bendable_feature else "xy")
record_file = "training_iteration.txt"
if REUSE_WEIGHT:
    training_num = increment_number_in_file(record_file, clear=False)
    file_identifier += ("_" + str(training_num))
else:
    increment_number_in_file(record_file, clear=True)


# last recorded acceleration
acc_l = None
acc_r = None


# Callback function for ROS subscriber
def ros_accel_callback_L(data):
    global acc_l
    acc_l = data.values


def ros_accel_callback_R(data):
    global acc_r
    acc_r = data.values


def get_trajectory_waypoint1(t):
    # Parameters
    R = 0.2     # length variation in xy-plane
    R_c = 0.4    # center of variation
    omega = 0.4*np.pi  # Frequency of length variation
    A = 0.05      # Amplitude of z oscillation 1
    kappa = 1.1  # Frequency of z oscillation 1
    rho = 0.8    # Frequency of z oscillation 2
    B = 0.0     # Rate of linear ascent in z-direction 
    C = -0.05     # Linear Translation rate in x
    y_offset = 0.0

    x = 2.0
    dot_x = 0.0
    y = R * np.cos(omega * t) + R_c
    dot_y = -omega*R*np.sin(omega*t)
    z_R = A * np.sin(kappa * t) + B * t
    dot_z_R  = kappa*A*np.cos(kappa*t) + B
    z_L = A * np.sin(rho * t) + B * t
    dot_z_L  = rho*A*np.cos(rho*t) + B        

    current_desired_right = np.array([x + C*t, y + y_offset, z_R + 1.0])
    current_desired_left = np.array([x + C*t, -y + y_offset, z_L + 1.0])
    current_desired_v_right = np.array([dot_x + C, dot_y,  dot_z_R])
    current_desired_v_left = np.array([ -dot_x + C, -dot_y,dot_z_L])

    if t > 120:
        return None
    return (current_desired_left, current_desired_right, 
            current_desired_v_left, current_desired_v_right, 
            0.0, 0.0,
            [0, 0, 0.0], [0, 0, 0.0])


def get_trajectory_waypoint2(t):
    # Parameters
    R = 0.18     # radius variation in xy-plane 1
    R_c = 0.42    # center of variation 1
    omega = 0.15*np.pi  # Frequency of radius variation
    Omega = 0.0*np.pi    # Angular speed in xy-plane
    A = 0.15      # Amplitude of z oscillation 1
    kappa = 0.9  # Frequency of z oscillation 1
    rho = 0.8    # Frequency of z oscillation 2
    phi_1 = 0  # initial phase of z oscillation 1
    phi_2 = 0  # initial phase of z oscillation 2
    B = 0.01     # Rate of linear ascent in z-direction 
    C = -0.05     # Linear Translation rate in x

    x = (R * np.cos(omega * t) + R_c)* np.cos(Omega * t)
    dot_x = -omega*R*np.sin(omega*t)*np.cos(Omega*t) - Omega*(R*np.cos(omega*t) + R_c)*np.sin(Omega*t)
    y = (R * np.cos(omega * t) + R_c) * np.sin(Omega * t)
    dot_y = -omega*R*np.sin(omega*t)*np.sin(Omega*t) + Omega*(R*np.cos(omega*t) + R_c)*np.cos(Omega*t)        
    z_R = A * np.sin(kappa * t + phi_1) + B * t
    dot_z_R  = kappa*A*np.cos(kappa*t + phi_1) + B
    z_L = A * np.sin(rho * t + phi_2) + B * t
    dot_z_L  = rho*A*np.cos(rho*t + phi_2) + B        

    current_desired_right = np.array([x + 0.5 + C*t, y, z_R + 0.7])
    current_desired_left = np.array([-x + 0.5 + C*t, -y, z_L + 0.7])
    current_desired_v_right = np.array([dot_x + C, dot_y,  dot_z_R])
    current_desired_v_left = np.array([ -dot_x + C, -dot_y,dot_z_L])

    return (current_desired_left, current_desired_right, 
            current_desired_v_left, current_desired_v_right, 
            Omega*t, Omega*t,
            [0, 0, Omega], [0, 0, Omega])


def get_trajectory_waypoint3(t):
    global INIT_HEIGHT, X_INDENT, start_x, start_z, end_x, end_z, part, T1, T2, T3, T4
    # piecewise function of four parts:
    # t < T1: go up and oscillate y_P, z_P
    # T1 <= t < T2: go down and decrease r
    # T2 <= t < T3: extend, forward, and land
    # Parameters
    # the trajectory has two aspects:
    # - task (go to a position)
    # - exploration (to make sure phi is PE)
    # find the part of trajectory the robots are suppose to follow
    if t < T1:
        # task-related params
        Z = 1.8                         # highest point
        X = -1.5
        # mark the transition
        if part == 0:
            part = 1
            start_z = INIT_HEIGHT
            end_x = X
            end_z = Z               # X-axis of the highest point 
        B = (end_z - start_z)/T1    # Rate of linear ascent in z-direction 
        C = end_x/T1                # Linear Translation rate in x
        # exploration-related params
        R = 0.175                   # length variation in the first part
        R_c = 0.425                 # center of variation
        omega = np.pi*0.55          # Radii Frequency of length variation
        A = 0.15                    # Amplitude of z oscillation
        kappa = np.pi/4             # Frequency of z oscillation 1
        rho = np.pi/5               # Frequency of z oscillation 2

        y = R * np.cos(omega * t) + R_c
        dot_y = -omega*R*np.sin(omega*t)
        z_R = A * np.sin(kappa * t) + B * t
        dot_z_R  = kappa*A*np.cos(kappa*t) + B
        z_L = A * np.sin(rho * t) + B * t
        dot_z_L  = rho*A*np.cos(rho*t) + B       

        vel_mult = 1.0
        if t < 0.5:
            vel_mult = 0.0
        current_desired_right = np.array([C*t + X_INDENT + ENV_OFFSET, y, z_R + start_z])
        current_desired_left = np.array([C*t + X_INDENT + ENV_OFFSET, -y, z_L + start_z])
        current_desired_v_right = vel_mult * np.array([C, dot_y, dot_z_R])
        current_desired_v_left = vel_mult * np.array([C, -dot_y, dot_z_L])
        
    elif t < T2:
        # task-related params
        Z = 0.8                     
        X = -2.4                    
        R = 0.25    # the desired magnitude of the end-point displacement
        # mark the transition
        if part == 1:
            part = 2
            start_x = end_x
            start_z = end_z
            end_x = X
            end_z = Z
        
        B = (end_z - start_z)/(T2 - T1) # Rate of linear ascent in z-direction 
        C = (end_x - start_x)/(T2 - T1) # Linear Translation rate in x
        y = R 
        z_R = B * (t - T1)
        dot_z_R  = B
        z_L = B * (t - T1)
        dot_z_L  = B        

        # no exploration
        vel_mult = 1.0
        if t < T1 + 0.5:
            vel_mult = 0.0
        current_desired_right = np.array([start_x + C*(t - T1) + X_INDENT + ENV_OFFSET, y, z_R + start_z])
        current_desired_left = np.array([start_x + C*(t - T1) + X_INDENT + ENV_OFFSET, -y, z_L + start_z])
        current_desired_v_right = vel_mult * np.array([C, 0, dot_z_R])
        current_desired_v_left = vel_mult * np.array([C, 0, dot_z_L])
    
    elif t < T3:
        # task-related params
        Z = 0.8                     
        X = -3.6
        R = 0.25    # the desired magnitude of the end-point displacement
                    
        # mark the transition
        if part == 2:
            part = 3
            start_x = end_x
            start_z = end_z
            end_x = X
            end_z = Z
        
        B = (end_z - start_z)/(T3 - T2) # Rate of linear ascent in z-direction
        C = 2.0*(end_x - start_x)/(T3 - T2) # Linear Translation rate in x
        y = R 
        z_R = B * (t - T2)
        dot_z_R  = B
        z_L = B * (t - T2)
        dot_z_L  = B        

        # no exploration
        vel_mult = 1.0
        if t < T2 + 0.5:
            vel_mult = 0.0
        
        if t < T2 + 0.5*(T3 - T2):
            x_pos = start_x + C*(t - T2)
            x_vel = C
        else:
            x_pos = end_x
            x_vel = 0.0
        current_desired_right = np.array([x_pos + X_INDENT + ENV_OFFSET, y, z_R + start_z])
        current_desired_left = np.array([x_pos + X_INDENT + ENV_OFFSET, -y, z_L + start_z])
        current_desired_v_right = vel_mult * np.array([x_vel, 0, dot_z_R])
        current_desired_v_left = vel_mult * np.array([x_vel, 0, dot_z_L])
    
    elif t < T4:
        # task-related params
        Z = 0.8                     
        X = -3.6
        R = 0.25 + (0.6 - 0.25)*(t - T3)/(T4 - T3)    # the desired magnitude of the end-point displacement
                    
        # mark the transition
        if part == 3:
            part = 4
            start_x = end_x
            start_z = end_z
            end_x = X
            end_z = Z
        
        B = (end_z - start_z)/(T4 - T3) # Rate of linear ascent in z-direction 
        C = (end_x - start_x)/(T4 - T3) # Linear Translation rate in x
        y = R 
        z_R = B * (t - T3)
        dot_z_R  = B
        z_L = B * (t - T3)
        dot_z_L  = B        

        # no exploration
        vel_mult = 1.0
        if t < T3 + 0.5:
            vel_mult = 0.0
        current_desired_right = np.array([start_x + C*(t - T3) + X_INDENT + ENV_OFFSET, y, z_R + start_z])
        current_desired_left = np.array([start_x + C*(t - T3) + X_INDENT + ENV_OFFSET, -y, z_L + start_z])
        current_desired_v_right = vel_mult * np.array([C, 0.0, dot_z_R])
        current_desired_v_left = vel_mult * np.array([C, 0.0, dot_z_L])
    
    else:
        return None

    return (current_desired_left, current_desired_right, 
            current_desired_v_left, current_desired_v_right, 
            0.0, 0.0,
            [0, 0, 0.0], [0, 0, 0.0])

# y_record_l = []
# y_record_r = []

# dy_record_l = []
# dy_record_r = []

def get_trajectory_waypoint4(t):
    global start_x, start_y, start_z, end_x, end_y, end_z, part #, y_record_r, y_record_l, dy_record_l, dy_record_r
    # piecewise function of two parts:
    # t < T1: exploration
    # - exploration: Similar to trajectory 2, but flying higher to avoid collision and no rotation
    if t < T1:
        # Parameters
        Beta = 2*np.pi/T1       # angular frequency in the x-plane motion
        R_v = 0.15              # radius variation in xy-plane 1
        R_c = 0.40              # center of variation 1
        omega = 2*np.pi/T1      # Frequency of radius variation
        Omega = 2*np.pi/T1      # Angular speed in xy-plane
        A = 0.10                 # Amplitude of z oscillation 1
        kappa = np.pi/5         # Frequency of z oscillation 1
        rho = np.pi/4           # Frequency of z oscillation 2
        # mark the transition
        if part == 0:
            part = 1
            print("phase", part)
            start_x = X_RADIUS + X_OFFSET
            start_z = INIT_HEIGHT
            end_x = X_RADIUS + X_OFFSET
            end_z = INIT_HEIGHT
        
        R_n = R_c + R_v*np.cos(omega*t)
        x_c = X_RADIUS*np.cos(Beta*t) + X_OFFSET
        y_c = Y_RADIUS*np.sin(2*Beta*t)
        x_offset_right = -R_n*np.sin(Omega*t)
        y_offset_right = R_n*np.cos(Omega*t)
        x_offset_left = -x_offset_right
        y_offset_left = -y_offset_right

        z_r = A * np.sin(kappa * t) + INIT_HEIGHT
        dot_z_r  = kappa*A*np.cos(kappa*t)
        z_l = A * np.sin(rho * t) + INIT_HEIGHT
        dot_z_l  = rho*A*np.cos(rho*t)  
        x_r = x_c + x_offset_right
        dot_x_r = -Beta*X_RADIUS*np.sin(Beta*t) - Omega*R_n*np.cos(Omega*t) + omega*np.sin(Omega*t)*R_v*np.sin(omega*t)
        x_l = x_c + x_offset_left
        dot_x_l = -Beta*X_RADIUS*np.sin(Beta*t) + Omega*R_n*np.cos(Omega*t) - omega*np.sin(Omega*t)*R_v*np.sin(omega*t)
        y_r = y_c + y_offset_right
        dot_y_r = 2*Beta*Y_RADIUS*np.cos(2*Beta*t) - Omega*R_n*np.sin(Omega*t) - omega*np.cos(Omega*t)*R_v*np.sin(omega*t)
        y_l = y_c + y_offset_left
        dot_y_l = 2*Beta*Y_RADIUS*np.cos(2*Beta*t) + Omega*R_n*np.sin(Omega*t) + omega*np.cos(Omega*t)*R_v*np.sin(omega*t)

        # y_record_l.append(y_l)
        # y_record_r.append(y_r)
        # dy_record_l.append(dot_y_l)
        # dy_record_r.append(dot_y_r)
        
        current_desired_right = np.array([x_r + ENV_OFFSET, y_r, z_r])
        current_desired_left = np.array([x_l + ENV_OFFSET, y_l, z_l])
        current_desired_v_right = np.array([dot_x_r, dot_y_r, dot_z_r])
        current_desired_v_left = np.array([dot_x_l, dot_y_l, dot_z_l])
        vel_mult = 1.0
        if t < 0.1:
            vel_mult = 0.0
        return (current_desired_left, current_desired_right, 
                vel_mult*current_desired_v_left, vel_mult*current_desired_v_right, 
                0.0*Omega*t, 0.0*Omega*t,
                [0, 0, 0.0*vel_mult*Omega], [0, 0, 0.0*vel_mult*Omega])

    elif t < T2:
        # T1 <= t < T2: first exploitation
        # Parameters
        R_v = 0.20                  # radius variation in xy-plane 1
        R_c = 0.40                  # center of variation 1
        omega = 2*np.pi/(T2 - T1)   # Angular frequency of radius variation
        # mark the transition
        if part == 1:
            part = 2
            print("phase", part)
            start_x = end_x
            start_z = end_z
            start_y = end_y
            end_x = start_x + X_OFFSET_FIRST_OBSTACLE
            end_z = HEIGHT_FIRST_OBSTACLE
            end_y = start_y + Y_OFFSET_FIRST_OBSTACLE
        
        A = (end_z - start_z)/(T2 - T1) # velocity in z-axis
        B = (end_x - start_x)/(T2 - T1) # velocity in x-axis
        C = (end_y - start_y)/(T2 - T1) # velocity in y-axis
        
        R_n = R_c + R_v*np.cos(omega*(t - T1))
        x_c = start_x + B*(t - T1)
        y_c = C*(t - T1) + start_y
        x_offset_right = 0.0
        y_offset_right = R_n
        x_offset_left = 0.0
        y_offset_left = -y_offset_right

        z_r = A*(t - T1) + start_z
        dot_z_r  = A
        z_l = A*(t - T1) + start_z
        dot_z_l  = A
        x_r = x_c + x_offset_right
        dot_x_r = B
        x_l = x_c + x_offset_left
        dot_x_l = B
        y_r = y_c + y_offset_right
        dot_y_r = -omega*R_v*np.sin(omega*(t - T1)) + C
        y_l = y_c + y_offset_left
        dot_y_l = omega*R_v*np.sin(omega*(t - T1)) + C

        # y_record_l.append(y_l)
        # y_record_r.append(y_r)
        # dy_record_l.append(dot_y_l)
        # dy_record_r.append(dot_y_r)
        
        current_desired_right = np.array([x_r + ENV_OFFSET, y_r, z_r])
        current_desired_left = np.array([x_l + ENV_OFFSET, y_l, z_l])
        current_desired_v_right = np.array([dot_x_r, dot_y_r, dot_z_r])
        current_desired_v_left = np.array([dot_x_l, dot_y_l, dot_z_l])

        # ensuring the numerical stability of the transition:
        vel_mult = 1.0
        if t < T1 + 0.1:
            vel_mult = 0.0
        return (current_desired_left, current_desired_right, 
                vel_mult * current_desired_v_left, vel_mult * current_desired_v_right, 
                0.0, 0.0,
                [0, 0, 0.0], [0, 0, 0.0])
    
    elif t < T3:
        # T2 <= t < T3: second exploitation - passing the first obstacle
        # Parameters
        R_v = 0.20                  # radius variation in xy-plane 1
        R_c = 0.40                  # center of variation 1
        Omega = np.pi/(T3 - T2)     # Angular frequency of y oscillation
        omega = 2*np.pi/(T3 - T2)   # Angular frequency of radius variation
        kappa = 2*np.pi/(T3 - T2)   # Frequency of z oscillation 1
        rho = -2*np.pi/(T3 - T2)    # Frequency of z oscillation 2
        # mark the transition
        if part == 2:
            part = 3
            print("phase", part)
            start_x = end_x
            start_z = end_z
            start_y = end_y
            end_x = start_x
            end_z = end_z
            end_y = -start_y
        
        A = 0.15 # oscillation in z-axis
        
        R_n = R_c + R_v*np.cos(omega*(t - T2))
        x_c = start_x
        y_c = start_y*np.cos(Omega*(t - T2))
        y_offset_right = R_n
        y_offset_left = -y_offset_right

        z_r = A * np.sin(kappa*(t - T2)) + start_z
        dot_z_r  = kappa*A*np.cos(kappa*(t - T2))
        z_l = A * np.sin(rho*(t - T2)) + start_z
        dot_z_l  = rho*A*np.cos(rho*(t - T2))  
        x_r = x_c
        dot_x_r = 0.0
        x_l = x_c
        dot_x_l = 0.0
        y_r = y_c + y_offset_right
        dot_y_r = -start_y*Omega*np.sin(Omega*(t - T2)) - omega*R_v*np.sin(omega*(t - T2))
        y_l = y_c + y_offset_left
        dot_y_l = -start_y*Omega*np.sin(Omega*(t - T2)) + omega*R_v*np.sin(omega*(t - T2))

        # y_record_l.append(y_l)
        # y_record_r.append(y_r)
        # dy_record_l.append(dot_y_l)
        # dy_record_r.append(dot_y_r)
        
        current_desired_right = np.array([x_r + ENV_OFFSET, y_r, z_r])
        current_desired_left = np.array([x_l + ENV_OFFSET, y_l, z_l])
        current_desired_v_right = np.array([dot_x_r, dot_y_r, dot_z_r])
        current_desired_v_left = np.array([dot_x_l, dot_y_l, dot_z_l])

        vel_mult = 1.0
        if t < T2 + 0.1:
            vel_mult = 0.0
        return (current_desired_left, current_desired_right, 
                vel_mult * current_desired_v_left, vel_mult * current_desired_v_right, 
                0.0, 0.0,
                [0, 0, 0.0], [0, 0, 0.0])
    elif t < T4:
        # T3 <= t < T4: third exploitation - passing the second obstacle
        # Parameters
        # Parameters
        R_v = 0.15                  # radius variation in xy-plane 1
        R_c = 0.45                  # center of variation 1
        omega = np.pi/(2*(T4 - T3))   # Frequency of radius variation
        Omega = np.pi/(2*(T4 - T3)) # Angular speed in xy-plane
        # mark the transition
        if part == 3:
            part = 4
            print("phase", part)
            start_x = end_x
            start_z = end_z
            start_y = end_y
            end_x = start_x + X_OFFSET_SECOND_OBSTACLE
            end_z = HEIGHT_SECOND_OBSTACLE
            end_y = 0.0
        
        A = (end_z - start_z)/(T4 - T3)         # velocity in z-axis
        B = (end_x - start_x)/(T4 - T3)         # velocity in x-axis
        C = Y_OFFSET_SECOND_OBSTACLE/(T4- T3)   # velocity in y-axis
        
        R_n = R_c + R_v*np.cos(omega*(t - T3))
        x_c = start_x + B*(t - T3)
        y_c = start_y + C*(t - T3)
        x_offset_right = -R_n*np.sin(Omega*(t - T3))
        y_offset_right = R_n*np.cos(Omega*(t - T3))
        x_offset_left = -x_offset_right
        y_offset_left = -y_offset_right

        z_r = start_z + A*(t - T3)
        dot_z_r  = A
        z_l = start_z + A*(t - T3)
        dot_z_l  = A
        x_r = x_c + x_offset_right
        dot_x_r = B + omega*np.sin(Omega*(t - T3))*R_v*np.sin(omega*(t - T3)) - R_n*Omega*np.cos(Omega*(t - T3))
        x_l = x_c + x_offset_left
        dot_x_l = B - omega*np.sin(Omega*(t - T3))*R_v*np.sin(omega*(t - T3)) + R_n*Omega*np.cos(Omega*(t - T3))
        y_r = y_c + y_offset_right
        dot_y_r = C - omega*np.cos(Omega*(t - T3))*R_v*np.sin(omega*(t - T3)) - R_n*Omega*np.sin(Omega*(t - T3))
        y_l = y_c + y_offset_left
        dot_y_l = C + omega*np.cos(Omega*(t - T3))*R_v*np.sin(omega*(t - T3)) + R_n*Omega*np.sin(Omega*(t - T3))

        # y_record_l.append(y_l)
        # y_record_r.append(y_r)
        # dy_record_l.append(dot_y_l)
        # dy_record_r.append(dot_y_r)
        
        current_desired_right = np.array([x_r + ENV_OFFSET, y_r, z_r])
        current_desired_left = np.array([x_l + ENV_OFFSET, y_l, z_l])
        current_desired_v_right = np.array([dot_x_r, dot_y_r, dot_z_r])
        current_desired_v_left = np.array([dot_x_l, dot_y_l, dot_z_l])

        vel_mult = 1.0
        if t < T3 + 0.1:
            vel_mult = 0.0
        return (current_desired_left, current_desired_right, 
                vel_mult * current_desired_v_left, vel_mult * current_desired_v_right, 
                0.0*Omega*(t - T3), 0.0*Omega*(t - T3),
                [0, 0, 0.0*vel_mult * Omega], [0, 0, 0.0*vel_mult * Omega])
    
    elif t < T5:
        # T3 <= t < T4: third exploitation - passing the second obstacle
        # Parameters
        # Parameters
        R_v = 0.15                          # radius variation in xy-plane 1
        R_c = 0.45                          # center of variation 1
        omega = 3*np.pi/(2*(T5 - T4))       # Frequency of radius variation
        Omega = -np.pi/(2*(T5 - T4))        # Angular speed in xy-plane
        phase = np.pi/2
        # mark the transition
        if part == 4:
            part = 5
            print("phase", part)
            start_x = end_x
            start_z = end_z
            start_y = end_y
            end_x = X_OFFSET - X_RADIUS
            end_z = HEIGHT_SECOND_OBSTACLE
            end_y = 0.0
        
        B = (end_x - start_x)/(T5 - T4) # velocity in x-axis
        R_n = R_c + R_v*np.cos(omega*(t - T4) + phase)
        x_c = start_x + B*(t - T4)
        y_c = 0.0
        x_offset_right = -R_n*np.sin(Omega*(t - T4) + phase)
        y_offset_right = R_n*np.cos(Omega*(t - T4) + phase)
        x_offset_left = -x_offset_right
        y_offset_left = -y_offset_right

        z_r = start_z
        dot_z_r  = 0.0
        z_l = start_z
        dot_z_l  = 0.0
        x_r = x_c + x_offset_right
        dot_x_r = B - Omega*R_n*np.cos(Omega*(t - T4) + phase) + omega*np.sin(Omega*(t - T4) + phase)*R_v*np.sin(omega*(t - T4) + phase)
        x_l = x_c + x_offset_left
        dot_x_l = B + Omega*R_n*np.cos(Omega*(t - T4) + phase) - omega*np.sin(Omega*(t - T4) + phase)*R_v*np.sin(omega*(t - T4) + phase)
        y_r = y_c + y_offset_right
        dot_y_r = -omega*np.cos(Omega*(t - T4) + phase)*R_v*np.sin(omega*(t - T4) + phase) - Omega*np.sin(Omega*(t - T4) + phase)*R_n
        y_l = y_c + y_offset_left
        dot_y_l = -omega*np.cos(Omega*(t - T4) + phase)*R_v*np.sin(omega*(t - T4) + phase) + Omega*np.sin(Omega*(t - T4) + phase)*R_n

        # y_record_l.append(y_l)
        # y_record_r.append(y_r)
        # dy_record_l.append(dot_y_l)
        # dy_record_r.append(dot_y_r)

        current_desired_right = np.array([x_r + ENV_OFFSET, y_r, z_r])
        current_desired_left = np.array([x_l + ENV_OFFSET, y_l, z_l])
        current_desired_v_right = np.array([dot_x_r, dot_y_r, dot_z_r])
        current_desired_v_left = np.array([dot_x_l, dot_y_l, dot_z_l])

        vel_mult = 1.0
        if t < T4 + 0.1:
            vel_mult = 0.0
        return (current_desired_left, current_desired_right, 
                vel_mult * current_desired_v_left, vel_mult * current_desired_v_right, 
                0.0*(phase/2 + Omega*(t - T4)), 0.0*(phase/2 + Omega*(t - T4)),
                [0, 0, 0.0*vel_mult*Omega], [0, 0, 0.0*vel_mult*Omega])
    else:
        return None

trajectory = get_trajectory_waypoint1  # select trajectory
if trajectory is get_trajectory_waypoint3:
    ENV_OFFSET = 1.1
    start_x, start_z, end_x, end_z = 0.0, 0.0, 0.0, 0.0
    T1 = 20
    T2 = 30
    T3 = 50
    T4 = 60
    X_INDENT = 2.0 # to correct the shifted starting position
    part = 0
elif trajectory is get_trajectory_waypoint4:
    ENV_OFFSET = 1.15
    INIT_HEIGHT, X_OFFSET, X_RADIUS, Y_RADIUS = 1.8, 0.2, 1.8, 0.2
    X_OFFSET_FIRST_OBSTACLE, HEIGHT_FIRST_OBSTACLE, Y_OFFSET_FIRST_OBSTACLE = -1.6, 0.7, 1.2
    X_OFFSET_SECOND_OBSTACLE, HEIGHT_SECOND_OBSTACLE, Y_OFFSET_SECOND_OBSTACLE = -1.5, 0.75, 1.2
    T1, T2, T3, T4, T5 = 40, 50, 60, 80, 100
    start_x, end_x, start_y, end_y, start_z, end_z = 0.0, X_RADIUS + X_OFFSET, 0.0, 0.0, 0.0, INIT_HEIGHT
    part = 0


def move_robots_with_velocity(allcfs, positions, velocities, yaws, omegas, accs):
    for cf, pos, vel, yaw, omega, acc in zip(allcfs.crazyflies, positions, velocities, yaws, omegas, accs):
        cf.cmdFullState(pos, vel, MASS_MULTIPLIER*acc, yaw, omega)


CAP = False
MIN_COV_VAL, MAX_COV_VAL = -2, 2
R_0 = 200.0 # the upperbound of the covariance matrix norm: ra 200
# Recursive Least Squares class
class rls_estimate:
    def __init__(self, order=3, lambda_forgetting=0.01, initial_covariance=2.0, weights=None, Ps=None):
        self.order = order           # order of polynomial feature
        self.ff = lambda_forgetting  # forgetting factor for RLS update
        if weights:
            self.W_L = weights[0]
            self.W_R = weights[1]
            self.P_L = Ps[0]
            self.P_R = Ps[1]
        else:
            if bendable_feature:
                self.W_L = np.zeros(shape=(4*order+1, 2)) # zero initial weight matrix
                self.W_R = np.zeros(shape=(4*order+1, 2)) # zero initial weight matrix
            else:
                self.W_L = np.zeros(shape=(4*order+1, 3)) # zero initial weight matrix
                self.W_R = np.zeros(shape=(4*order+1, 3)) # zero initial weight matrix
            self.P_L = np.eye(4*order+1) * initial_covariance # Large initial covariance
            self.P_R = np.eye(4*order+1) * initial_covariance # Large initial covariance
        self.last_F_L = np.zeros((3,))
        self.last_F_R = np.zeros((3,))
    
    # get the rotation from the world reference to the rod plane
    def get_Rp_alpha_yp(self, r):
        z_p = np.array([0, 0, 1])
        y_p = np.cross(r, z_p)/np.linalg.norm(np.cross(r, z_p))
        x_p = np.cross(y_p, z_p)
        R_p = np.column_stack((x_p, y_p, z_p))
        alpha = np.arctan2(r.dot(z_p), r.dot(x_p))
        return R_p, alpha, y_p

    def rls_update(self, phi, R_p, f_ext, dt, side):
        global force_estimation_error_L, force_estimation_error_R
        if side == 'R':
            epsilon = R_p.T.dot(f_ext) - self.W_R.T.dot(phi)
            W_dot = self.P_R @ np.outer(phi, epsilon)
            if np.linalg.norm(self.P_R) < R_0:
                P_dot = self.ff * self.P_R - self.P_R @ np.outer(phi, phi) @ self.P_R
            else:
                P_dot = np.zeros_like(self.P_R)
            if CAP:
                W_dot = np.clip(W_dot, MIN_COV_VAL, MAX_COV_VAL)
                P_dot = np.clip(P_dot, MIN_COV_VAL, MAX_COV_VAL)
            self.W_R += W_dot*dt
            self.P_R += P_dot*dt
            force_estimation_error_R.append(epsilon)
        elif side == 'L':
            epsilon = R_p.T.dot(f_ext) - self.W_L.T.dot(phi)
            W_dot = self.P_L @ np.outer(phi, epsilon)
            if np.linalg.norm(self.P_L) < R_0:
                P_dot = self.ff * self.P_L - self.P_L @ np.outer(phi, phi) @ self.P_L
            else:
                P_dot = np.zeros_like(self.P_L)
            if CAP:
                W_dot = np.clip(W_dot, MIN_COV_VAL, MAX_COV_VAL)
                P_dot = np.clip(P_dot, MIN_COV_VAL, MAX_COV_VAL)
            self.W_L += W_dot*dt
            self.P_L += P_dot*dt
            force_estimation_error_L.append(epsilon)
        else:
            assert False
        
        # print(P_dot)
    
    # Recursive Least Squares (RLS) update with bendable features
    def rls_update_bendable(self, phi, hat_r, hat_n, f_ext, dt, side):
        global force_estimation_error_L, force_estimation_error_R
        if side == 'R':
            epsilon = np.array([hat_r.dot(f_ext), hat_n.dot(f_ext)]) - self.W_R.T.dot(phi)
            W_dot = self.P_R @ np.outer(phi, epsilon)
            if np.linalg.norm(self.P_R) < R_0:
                P_dot = self.ff * self.P_R - self.P_R @ np.outer(phi, phi) @ self.P_R
            else:
                P_dot = np.zeros_like(self.P_R)
            if CAP:
                W_dot = np.clip(W_dot, MIN_COV_VAL, MAX_COV_VAL)
                P_dot = np.clip(P_dot, MIN_COV_VAL, MAX_COV_VAL)
            self.W_R += W_dot*dt
            self.P_R += P_dot*dt
            force_estimation_error_R.append(epsilon)
        elif side == 'L':
            epsilon = np.array([hat_r.dot(f_ext), hat_n.dot(f_ext)]) - self.W_L.T.dot(phi)
            W_dot = self.P_L @ np.outer(phi, epsilon)
            if np.linalg.norm(self.P_L) < R_0:
                P_dot = self.ff * self.P_L - self.P_L @ np.outer(phi, phi) @ self.P_L
            else:
                P_dot = np.zeros_like(self.P_L)
            if CAP:
                W_dot = np.clip(W_dot, MIN_COV_VAL, MAX_COV_VAL)
                P_dot = np.clip(P_dot, MIN_COV_VAL, MAX_COV_VAL)
            self.W_L += W_dot*dt
            self.P_L += P_dot*dt
            force_estimation_error_L.append(epsilon)
        else:
            assert False
        
        # print(P_dot)
    
    def estimate(self, R_p, phi, side):
        if side == 'R':
            return R_p @ self.W_R.T @ phi
        elif side == 'L':
            return R_p @ self.W_L.T @ phi
        else:
            print("BAD params!")
            return np.zeros((3, ))


def main():
    global pos_errors_L, pos_errors_R, rate, DT, actual_pos_L, actual_pos_R, actual_pos_L, actual_pos_R, trajectory
    K_p = 1.0
    swarm = Crazyswarm("../launch/rodcrazyflies.yaml")
    if REUSE_WEIGHT and force_estimation:
        with open('Weights.pkl', 'rb') as fp:
            Ps, weights = pickle.load(fp)
        estimator = rls_estimate(weights=weights, Ps=Ps)
    else:
        estimator = rls_estimate()
    # Subscribe to ROS topic for acceleration
    # rospy.Subscriber('/cf5/accs', GenericLogData, ros_accel_callback_L)
    # rospy.Subscriber('/cf6/accs', GenericLogData, ros_accel_callback_R)

    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # ellipsoid_radii = [0.13, 0.13, 0.13]
    totalDuration = 200

    # Initial setup, similar to before
    for cf in allcfs.crazyflies:
        # others = [other_cf for other_cf in allcfs.crazyflies if other_cf.id != cf.id]
        # cf.enableCollisionAvoidance(others, ellipsoid_radii) # let them clash and crash!
        cf.takeoff(targetHeight=INIT_HEIGHT, duration=5.0)

    timeHelper.sleep(2)  # Ensure that takeoff completes
    
    p_l, p_r, v_l, v_r, yaw_l, yaw_r, omega_l, omega_r = trajectory(0)
    #robot go to desire positon

    allcfs.crazyflies[0].goTo(p_l, 0, 5)
    allcfs.crazyflies[1].goTo(p_r, 0, 5)

    timeHelper.sleep(5.0)
    last_r = p_r - p_l

    # Start controlling the drones with velocity
    start_time = timeHelper.time()
    # try:
    while timeHelper.time() - start_time < totalDuration and swarm.input.checkIfAnyButtonIsPressed() == None:
        t = timeHelper.time() - start_time
        last_time = t
        res = trajectory(t)
        if res is None:
            break

        p_l, p_r, v_l, v_r, yaw_l, yaw_r, omega_l, omega_r = res
        des_pos_L.append(p_l)
        des_pos_R.append(p_r)

        # Collect desired positions and velocities
        positions = [p_l, p_r]
        velocities = [v_l, v_r]
        yaws = [yaw_l, yaw_r]
        omegas = [omega_l, omega_r]

        # get current robot position
        pos_l = np.array(allcfs.crazyflies[0].position())
        pos_r = np.array(allcfs.crazyflies[1].position())
        actual_pos_L.append(pos_l)
        actual_pos_R.append(pos_r)
        # vel_l = np.array(allcfs.crazyflies[0].velocity()) # not implemented w/ real robots
        # vel_r = np.array(allcfs.crazyflies[1].velocity()) # need to replace with log systems
        r_left_to_right = pos_r - pos_l
        dot_r_left_to_right = (r_left_to_right - last_r)/DT
        # print(dot_r_left_to_right)
        last_r = r_left_to_right
        r_mag = np.linalg.norm(r_left_to_right)*MAG_FEATURE
        dot_r_mag = np.linalg.norm(dot_r_left_to_right)*MAG_FEATURE

        pos_errors_L.append(p_l - pos_l)
        pos_errors_R.append(p_r - pos_r)

        # Move drones with velocity
        if force_estimation:
            # print(R_p @ estimator.W_L.T @ phi, R_p @ estimator.W_R.T @ phi)
            R_p, alpha, y_p = estimator.get_Rp_alpha_yp(r_left_to_right)
            r_P = R_p.T.dot(r_left_to_right)
            dot_r_P = R_p.T.dot(dot_r_left_to_right)
            f_ext_L = -K_p*(p_l - pos_l) + estimator.last_F_L
            f_ext_R = -K_p*(p_r - pos_r) + estimator.last_F_R
            force_gt_L.append(f_ext_L)
            force_gt_R.append(f_ext_R)

            phi = []
            if bendable_feature:
                for i in range(estimator.order):
                    phi.append(r_mag**(estimator.order - i))
                for i in range(estimator.order):
                    phi.append(dot_r_mag**(estimator.order - i))
                for i in range(estimator.order):
                    phi.append(np.cos((estimator.order - i)*alpha))
                for i in range(estimator.order):
                    phi.append(np.sin((estimator.order - i)*alpha))
                hat_r = r_left_to_right/r_mag
                # calculate the skew symmetric matrix of the unit vector
                C_matrix = np.array([[0, -y_p[2], y_p[1]],
                                    [y_p[2], 0, -y_p[0]],
                                    [-y_p[1], y_p[0], 0]])

                # https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula#Matrix_notation
                Rot = np.eye(3) + C_matrix*np.sin(np.pi/2) + C_matrix.dot(C_matrix)*(1 - np.cos(np.pi/2))
                hat_n = Rot.dot(hat_r)
            else:
                r_x = r_P[0]*MAG_FEATURE
                r_z = r_P[2]*MAG_FEATURE
                dot_rx = dot_r_P[0]*MAG_FEATURE
                dot_rz = dot_r_P[2]*MAG_FEATURE
                for i in range(estimator.order):
                    phi.append(r_x**(estimator.order - i))
                for i in range(estimator.order):
                    phi.append(r_z**(estimator.order - i))
                for i in range(estimator.order):
                    phi.append(dot_rx**(estimator.order - i))
                for i in range(estimator.order):
                    phi.append(dot_rz**(estimator.order - i))
            phi.append(1)
            phi = np.array(phi)

            if t < 2.0:
                pass
            else:
                if bendable_feature:
                    estimator.rls_update_bendable(phi, hat_r, hat_n, f_ext_L, DT, side="L")
                    estimator.rls_update_bendable(phi, hat_r, hat_n, f_ext_R, DT, side="R")
                else:
                    estimator.rls_update(phi, R_p, f_ext_L, DT, side="L")
                    estimator.rls_update(phi, R_p, f_ext_R, DT, side="R")

            if np.any(np.isnan(estimator.P_L)) or np.any(np.isnan(estimator.P_R)):
                assert False
            
            if bendable_feature:
                f_lambdas_L = estimator.W_L.T @ phi
                f_estimated_L = f_lambdas_L[0]*hat_r + f_lambdas_L[1]*hat_n
                f_lambdas_R = estimator.W_R.T @ phi
                f_estimated_R = f_lambdas_R[0]*hat_r + f_lambdas_R[1]*hat_n
                # print(f_lambdas_L.shape, f_lambdas_R)
                estimator.last_F_R = f_estimated_R
                estimator.last_F_L = f_estimated_L
                move_robots_with_velocity(allcfs, positions, velocities, yaws, omegas, accs=[-f_estimated_L, -f_estimated_R])
            else:
                f_b_R = estimator.W_R.T @ phi
                f_b_L = estimator.W_L.T @ phi
                estimator.last_F_R = R_p @ f_b_R
                # print(estimator.W_R.T @ phi)
                estimator.last_F_L = R_p @ f_b_L

                move_robots_with_velocity(allcfs, positions, velocities, yaws, omegas, accs=[-estimator.last_F_L, -estimator.last_F_R])
        else:
            move_robots_with_velocity(allcfs, positions, velocities, yaws, omegas, accs=[np.zeros((3,)), np.zeros((3,))])
        timeHelper.sleepForRate(rate)  # Short sleep to continue updating positions

    # Landing sequence
    allcfs.land(targetHeight=0.01, duration=5)
    timeHelper.sleep(5)
    print("Landing complete.")
    if force_estimation:
        with open('Weights.pkl', 'wb') as f:
            pickle.dump([[estimator.P_L, estimator.P_R], [estimator.W_L, estimator.W_R]], f)
    # except:
    #     print("Semi-emergency landing caused by numerical instability!")
    #     allcfs.land(targetHeight=0.01, duration=5)
    print(estimator.W_R.T @ phi)

if __name__ == "__main__":
    main()
    pos_errors_L = np.array(pos_errors_L)
    pos_errors_R = np.array(pos_errors_R)
    des_pos_L = np.array(des_pos_L)
    des_pos_R = np.array(des_pos_R)
    actual_pos_L = np.array(actual_pos_L)
    actual_pos_R = np.array(actual_pos_R)
    force_estimation_error_L = np.array(force_estimation_error_L)
    force_estimation_error_R = np.array(force_estimation_error_R)
    force_gt_L = np.array(force_gt_L)
    force_gt_R = np.array(force_gt_R)

    fig = plt.figure(figsize=(10, 6), frameon=True, dpi=200)
    ax = fig.add_subplot(111)
    x_values = [i * DT for i in range(len(pos_errors_L))]
    ax.plot(x_values, np.linalg.norm(pos_errors_L, axis=1), label='left')
    ax.plot(x_values, np.linalg.norm(pos_errors_R, axis=1), label='right')
    ax.set_ylim(0, 0.3)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.legend()
    plt.grid(True)
    # fig.show()
    # plt.show()
    plt.savefig("errors_" + file_identifier + ".pdf")

    # fig = plt.figure()
    # ax1 = fig.add_subplot(121)
    # ax1.plot(y_record_l)
    # ax1.plot(y_record_r)
    # ax2 = fig.add_subplot(122)
    # ax2.plot(dy_record_l)
    # ax2.plot(dy_record_r)
    # ax1.legend()
    # ax2.legend()
    # fig.show()
    # plt.show()

    if force_estimation:
        fig = plt.figure(figsize=(10, 6), frameon=True, dpi=200)
        ax = fig.add_subplot(111)
        x_values = [i * DT for i in range(len(force_estimation_error_L))]
        ax.plot(x_values, np.linalg.norm(force_estimation_error_L, axis=1), label='left')
        ax.plot(x_values, np.linalg.norm(force_estimation_error_R, axis=1), label='right')
        ax.set_ylim(0, 0.5)
        plt.xlabel('Time (s)')
        plt.ylabel('force_estimation_error (N)')
        plt.legend()
        plt.grid(True)
        # fig.show()
        # plt.show()
        plt.savefig("force_errors" + file_identifier + ".pdf")

        fig = plt.figure(figsize=(10, 6), frameon=True, dpi=200)
        ax = fig.add_subplot(111)
        x_values = [i * DT for i in range(len(force_gt_L))]
        ax.plot(x_values, np.linalg.norm(force_gt_L, axis=1), label='left')
        ax.plot(x_values, np.linalg.norm(force_gt_R, axis=1), label='right')
        ax.set_ylim(0, 1.5)
        plt.xlabel('Time (s)')
        plt.ylabel('ground truth force (N)')
        plt.legend()
        plt.grid(True)
        # fig.show()
        # plt.show()
        plt.savefig("force" + file_identifier + ".pdf".format("RLS"))
        with open('force_estimation_error_L' + file_identifier + '.pkl', 'wb') as f:
            pickle.dump(force_estimation_error_L, f)
        with open('force_estimation_error_R' + file_identifier + '.pkl', 'wb') as f:
            pickle.dump(force_estimation_error_R, f)
        with open('force_gt_L' + file_identifier + '.pkl', 'wb') as f:
            pickle.dump(force_gt_L, f)
        with open('force_gt_R' + file_identifier + '.pkl', 'wb') as f:
            pickle.dump(force_gt_R, f)

    with open('pos_err_L' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(pos_errors_L, f)
    with open('pos_err_R' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(pos_errors_R, f)
    with open('des_pos_L' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(des_pos_L, f)
    with open('des_pos_R' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(des_pos_R, f)
    with open('actual_pos_L' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(actual_pos_L, f)
    with open('actual_pos_R' + file_identifier + '.pkl', 'wb') as f:
        pickle.dump(actual_pos_R, f)



