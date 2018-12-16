#!/usr/bin/env python

import roslib;
roslib.load_manifest('kobuki_testsuite')
import rospy
from kobuki_msgs.msg import ButtonEvent
import subprocess
import time
import paramiko
import os


def button_event_callback(data):  # This function is run whenever someone pushes or releases the button.
    if data.state == ButtonEvent.RELEASED:
        state = "released"
    else:
        state = "pressed"
    if data.button == ButtonEvent.Button0:
        button = "B0"
    elif data.button == ButtonEvent.Button1:
        button = "B1"
    else:
        button = "B2"
    global robot_is_ready
    if button == "B0" and state == "pressed":
	robot_is_ready = True
        rospy.loginfo("Robot is ready.")

rospy.init_node("ssh_node")
robot_is_ready = False  # Initial value for statement.
rospy.loginfo("Robot is in standby mode, press B0 to initialize.")

# Run button_event_callback when button is pressed:
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_event_callback)

# Keep program running until ctrl+c is pressed:
rospy.spin()
