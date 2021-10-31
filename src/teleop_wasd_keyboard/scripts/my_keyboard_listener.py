#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import Twist
from pynput import keyboard


# -------------------------------------------------------------------------— Setting up
# ------------------------------------------------------------------ Keymap
valid_inputs = {
    # "letter":{"change":[vx, vy, wz], "state":None}

    # wasd controls linear motion
    'w':{"change":[ 1, 0, 0], "state":None},
    'a':{"change":[ 0,-1, 0], "state":None},
    's':{"change":[-1, 0, 0], "state":None},
    'd':{"change":[ 0, 1, 0], "state":None},

    # qe controls the angular motion
    'q':{"change":[ 0, 0, 1], "state":None},
    'e':{"change":[ 0, 0,-1], "state":None},

    # For braking, stoping the bot
    "space":{"change":[ 0, 0, 0], "state":None}
}

key_2_vel_idx = {
    'w':0,
    'a':1,
    's':0,
    'd':1,
    'q':2,
    'e':2,
}

# ------------------------------------------------------------------ Robot parameters
Vel = [0, 0, 0]  
dt  = 0.1 
max_acc  = 1
max_vel  = 10

# ------------------------------------------------------------------------- Functions
# ------------------------------------------------------------------ update_Velocity
def update_Velocity():
    """
    Updates the velocity variable based on the state of control keys
    """
    for key in valid_inputs.keys():
        if valid_inputs[key]["state"] == "down":
            if key == "space":
                calc_BrakeTrajectory()

            else:
                calc_AccelerateTrajectory(key)

# ------------------------------------------------------------------ assign_AccelerateCommand
def calc_AccelerateTrajectory(key):
    """
    Calculates the velocity when accelerate command is registered and 
    assigns it to the appropriate component of the velocity list

    Parameters
    ----------
    key : int
        The index of the velocity list
    """

    vel_idx  = key_2_vel_idx[key]
    sign     = valid_inputs[key]["change"][vel_idx]
    cur_vel  = Vel[vel_idx]
    next_vel = cur_vel + max_acc*dt*sign

    if abs(next_vel) > max_vel:
        next_vel = max_vel*sign

    Vel[vel_idx] = next_vel

# ------------------------------------------------------------------ assign_BrakeCommand
def calc_BrakeTrajectory():
    """
    Calculates the velocity when the brake command is registered and
    assigns it to the velcoity list
    """
    for idx in range(3):
        abs_change_vel = abs(Vel[idx]) - max_acc*dt
        abs_change_vel = max(abs_change_vel, 0)
        vel_sign       = math.copysign(1, Vel[idx])
        Vel[idx]       = vel_sign*abs_change_vel


# ------------------------------------------------------------------ callback_respond_ToKeyPress
def callback_respond_ToKeyPress(key):
    """
    Callback function which is triggered when a key is pressed

    Parameters
    ----------
    key : str
        The key that is pressed

    Returns
    -------
    bool
        Should we stop listenning
    """
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        key_name = key.char  # single-char keys
    except:
        key_name = key.name  # other keys
    
    if key_name in valid_inputs.keys():
        valid_inputs[key_name]["state"] = "down"

# ------------------------------------------------------------------ callback_respond_ToKeyRelease
def callback_respond_ToKeyRelease(key):
    """    
    Callback function which is triggered when a key is released

    Parameters
    ----------
    key : str
        The key that is released

    Returns
    -------
    bool
        Should we stop listenning
    """
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        key_name = key.char  # single-char keys
    except:
        key_name = key.name  # other keys
    
    if key_name in valid_inputs.keys():
        valid_inputs[key_name]["state"] = "up"

# -------------------------------------------------------------------------— mane 
if __name__ == '__main__':
    # Initialise the listener
    listener = keyboard.Listener(on_press=callback_respond_ToKeyPress, on_release = callback_respond_ToKeyRelease)
    listener.start()  # start to listen on a separate thread
    # listener.join()   # add if main thread is polling self.keys
    
    # Initialising the rosnode
    rospy.init_node("my_teleop")
    pub  = rospy.Publisher("/my_cmd_topic", Twist, queue_size = 1)
    rate = rospy.Rate(1/dt) 

    # initialising the message
    msg        = Twist()
    vx, vy, wz = Vel 
    msg.linear.x  = vx
    msg.linear.y  = vy
    msg.linear.z  = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = wz

    # Running the node
    while not rospy.is_shutdown():
        # Calculate the velocity and assign it to the message
        update_Velocity()
        vx, vy, wz = Vel
        msg.linear.x  = vx
        msg.linear.y  = vy
        msg.angular.z = wz

        pub.publish(msg)
        rate.sleep()

        # Debug
        # rospy.loginfo(f'the velocity are vx: {vx}, vy: {vy}, wz: {wz}')
            
