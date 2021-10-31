#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import json

def callback_append_Message(msg):
    global Vel_list
    vx = msg.linear.x
    vy = msg.linear.y
    wz = msg.angular.z
    
    Vel_list["vx"].append(vx)
    Vel_list["vy"].append(vy)
    Vel_list["wz"].append(wz)


if __name__ == '__main__':
    rospy.init_node("Listener")
    sub = rospy.Subscriber("/wasd_cmd_topic", Twist, callback_append_Message)

    Vel_list = {"vx":[], "vy": [], "wz": []}
    
    rospy.spin()

    with open("out.json", "w") as file:
        json.dump(Vel_list, file, indent=4)

    rospy.loginfo("This node is shutting down")