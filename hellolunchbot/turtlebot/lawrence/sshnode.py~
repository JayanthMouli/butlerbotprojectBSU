"""

                       _oo0oo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      0\  =  /0
                    ___/`---'\___
                  .' \\|     |// '.
                 / \\|||  :  |||// \
                / _||||| -:- |||||- \
               |   | \\\  -  /// |   |
               | \_|  ''\---/''  |_/ |
               \  .-\__  '-'  ___/-. /
             ___'. .'  /--.--\  `. .'___
          ."" '<  `.___\_<|>_/___.' >' "".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `_.   \_ __\ /__ _/   .-` /  /
     =====`-.____`.___ \_____/___.-`___.-'=====

This code was written by Kevin Jin and Lawrence Kimsey in Fall of 2016 for the Butlerbot project. It acts as a node that
constantly connects to a server, checking for any requests for a delivery. It should first be started while in a dock,
so that it may run the undocking command at the start. After each delivery, it updates its location so that it may run
go_to.py with accurate arguments. After each delivery to a place that isn't a dock, the robot will enter standby mode.
The robot will exit standby mode when button b0 is pressed on the robot base.

IMPORTANT: ctrl+c sometimes does not cleanly exit out of this program! To mitigate this, a feature has been added where
the loop at the end will break when button b2 is pressed. Please use this to exit out of the code whenever possible.
If this does not work, press ctrl+shift+\ to force quit. If you find a fix for this, please put it in. 
"""

import roslib
roslib.load_manifest('kobuki_testsuite')
import rospy
from kobuki_msgs.msg import ButtonEvent
import subprocess
import time
import paramiko
import os
import subprocess


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
    # Set variables robot_is_ready and shut_down_robot based on button press:
    global robot_is_ready
    global shut_down_robot
    if button == "B0" and state == "pressed":  # Press B0 to send to next destination.
        robot_is_ready = True
        rospy.loginfo("Robot is ready for the next destination.")
    elif button == "B2" and state == "pressed":  # Press B2 to end program.
        rospy.loginfo("B2 pressed, robot will stop connecting to server soon.")
        shut_down_robot = True


def update_location():  # This function updates the value of current_position based on location
    global room
    global current_position
    global robot_is_docked

    # Set current_position based on last destination:
    if room[:6] == "mec202" or room[:7] == "cubicle":
        current_position = "mec202"
    elif room == "dock":
        current_position = "mec202"
        robot_is_docked = True  # Used to tell if robot should undock next run.
    elif room[:7] == "engr201":
        current_position = "engr201"
    elif room[:7] == "engr240":
        current_position = "engr240"
    else:
        current_position = "hallway"


rospy.init_node("ssh_node")

# Run button_event_callback when button is pressed:
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_event_callback)

# Initial values:
robot_is_ready = False  # Set to True if you don't want to start the robot by pressing B0.
shut_down_robot = False  # Don't change this. When this is switched to True, while loop ends.
robot_is_docked = True  # Set to False if the code is not started while in the dock.
current_position = "mec202"  # Set to whatever area the dock is in.

print("To send to next destination: press b0 on the robot base.")
print("To exit: press either b2 on the robot (prefered method) or ctrl+shift+\ ")

while not shut_down_robot:  # Loop repeats until someone presses B2!
    try:
        if robot_is_ready:  # Robot only works when someone presses B0!
            print('Getting list from server...')
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # this will automatically add the keys
            ssh.connect("tavernaweb", username="kjin", password="lunchbot")
            stdin, stdout, stderr = ssh.exec_command('python sqlsecond.py')  # test running the other code
            data = stdout.readlines()  # Read room list from server

            # Get room name from top of list, remove newline from end, and make lowercase:
            room = data[0][:-1].lower() 
            print("Received destination: " + room)
            time.sleep(2)

            # Obsolute block of code? Try uncommenting if something doesn't work anymore!
            ## example 2 : change ip address
            #print('changing ip address')
            #stdin, stdout, stderr = ssh.exec_command('sed -i -- s/' + '132.178.226.152' + '/' + '10.254.38.45' + '/g /etc/sysconfig/network-scripts/ifcfg-eth0')
            #print(stdout.readlines())
            #time.sleep(2)

            # If it goes anywhere other than the dock, someone will have to press B0 to make it move again:
            if room != "dock":
                robot_is_ready = False

            # Check if robot is trying to go to the dock when it's already docked. If not, start moving!
            if room != "dock" or (room == "dock" and not robot_is_docked):
                # if robot is docked, run undocking program:
                if robot_is_docked:
                    subprocess.call(" python ~/hellolunchbot/turtlebot/lawrence/undock.py", shell=True)
                    robot_is_docked = False

                # Give movement command.
                print("Running go_to.py %s %s " % (room, current_position))
                command = " python ~/hellolunchbot/turtlebot/lawrence/go_to.py " + room + " " + current_position
                subprocess.call(command, shell=True)

                # Update location-based variables using new location.
                update_location()
                time.sleep(5)
            else:  # If it's already docked, just wait 30 seconds for a new destination.
                print("Robot is already docked. Waiting 30 seconds for new destination...")
                time.sleep(30)

            print("To send to next destination: press B0 on the robot base.")
            print("To quit: press either B2 (prefered method) or ctrl+shift+\ ")

        else:
            print("Waiting for button press...")
            time.sleep(5)
    except KeyboardInterrupt:
        # quit
        sys.exit()

print("Node has successfully terminated. Goodbye!")

