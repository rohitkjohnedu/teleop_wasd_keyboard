#!/usr/bin/env python3
from pynput import keyboard
import rospy
from teleop_wasd_keyboard.msg import cmd_2D_robot_vel
import time
import math

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


def update_Velocity():
    for key in valid_inputs.keys():
        if valid_inputs[key]["state"] == "down":
            if key == "space":
                for idx in range(3):
                    abs_change_vel = abs(Vel[idx]) - max_acc*dt
                    abs_change_vel = max(abs_change_vel, 0)
                    vel_sign       = math.copysign(1, Vel[idx])
                    Vel[idx]       = vel_sign*abs_change_vel

            else:
                vel_idx  = key_2_vel_idx[key]
                sign     = valid_inputs[key]["change"][vel_idx]
                cur_vel  = Vel[vel_idx]
                next_vel = cur_vel + max_acc*dt*sign

                if abs(next_vel) > max_vel:
                    next_vel = max_vel*sign

                Vel[vel_idx] = next_vel


def callback_respond_ToKeyPress(key):
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        key_name = key.char  # single-char keys
    except:
        key_name = key.name  # other keys
    
    if key_name in valid_inputs.keys():
        valid_inputs[key_name]["state"] = "down"


def callback_respond_ToKeyrelease(key):
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        key_name = key.char  # single-char keys
    except:
        key_name = key.name  # other keys
    
    if key_name in valid_inputs.keys():
        valid_inputs[key_name]["state"] = "up"

if __name__ == '__main__':
    Vel = [0, 0, 0]   
    vx, vy, wz = Vel 
    dt  = 2
    max_acc  = 1
    max_vel  = 10
    listener = keyboard.Listener(on_press=callback_respond_ToKeyPress, on_release = callback_respond_ToKeyrelease)
    listener.start()  # start to listen on a separate thread
    # listener.join()   # add if main thread is polling self.keys
    
    
    rospy.init_node("my_teleop")
    pub  = rospy.Publisher("/my_cmd_topic", cmd_2D_robot_vel, queue_size = 1)
    rate = rospy.Rate(1/dt) 
    msg  = cmd_2D_robot_vel()

    msg.vx = vx
    msg.vy = vy
    msg.wz = wz
    while True:
        update_Velocity()
        vx, vy, wz = Vel
        msg.vx = vx
        msg.vy = vy
        msg.wz = wz
        pub.publish(msg)
        rate.sleep()

        # Debug
        # print(f'the velocity are vx: {vx}, vy: {vy}, wz: {wz}')
            
