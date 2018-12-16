import roslib;
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
    global robot_is_ready
    global shut_down_robot
    if button == "B0" and state == "pressed":
	robot_is_ready = True
        rospy.loginfo("Robot is ready for the next destination.")
    elif button == "B2" and state == "pressed":
        rospy.loginfo("B2 pressed, robot will stop connecting to server soon.")
        shut_down_robot = True

def update_location():  # This function updates the value of current_position based on location
    global room
    global current_position
    global robot_is_docked

    if room[:6] == "mec202" or room[:7] == "cubicle":
        current_position = "in"
    elif room == "dock":
        current_position = "in"
        robot_is_docked = True
    else: 
        current_position = "out"


rospy.init_node("ssh_node")

# Run button_event_callback when button is pressed:
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_event_callback)

# Initial values:
robot_is_ready = False  # Set to True if you don't want to start the robot by pressing B0.
shut_down_robot = False # Don't change this. When this is switched to True, while loop ends.
robot_is_docked = True  # Set to False if the code is not started while in the dock.
current_position = "in" # Set to "out" if robot starts outside of MEC202 office area.

print "To send to next destination: press b0 on the robot base."
print "To exit: press either b2 on the robot (prefered method) or ctrl+shift+\ "

while(not shut_down_robot):
    if(robot_is_ready):
        print 'enter ssh'
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy()) # this will automatically add the keys
        ssh.connect("tavernaweb", username="kjin", password="lunchbot")

        print 'sqlsecond below'
        stdin, stdout, stderr = ssh.exec_command('python sqlsecond.py') #test running the other code
        yolo = stdout.readlines()  #room name
        room = yolo[0]
        #room = "MEC202C"
	room = room[:-1].lower()
        print room
        time.sleep(2)
        # example 2 : change ip address
        print 'changing ip address'
        stdin, stdout, stderr = ssh.exec_command('sed -i -- s/'+'132.178.226.152'+'/'+'10.254.38.45'+'/g /etc/sysconfig/network-scripts/ifcfg-eth0')
        print stdout.readlines()
        time.sleep(2)

        # set robot to standby if it goes somewhere other than the dock:
        if room != "dock":
            robot_is_ready = False

        # check if robot is trying to go to the dock when it's already docked. If so, skip.
        if (not room == "dock") or (room == "dock" and not robot_is_docked):
            # if robot is docked, run undocking node:
            if robot_is_docked: 
                subprocess.call(" python ~/hellolunchbot/turtlebot/lawrence/undock.py", shell=True)
                robot_is_docked = False

            # give movement command, then update location-based variables.
            print "Running go_to.py %s %s " % (room, current_position)
            command = " python ~/hellolunchbot/turtlebot/lawrence/go_to.py " + room + " " + current_position 
            subprocess.call(command, shell=True)

        update_location()
        time.sleep(5)

        print "To send to next destination: press b0 on the robot base."
        print "To exit: press either b2 on the robot (prefered method) or ctrl+shift+\ "

    else:
        print "waiting for button press."
        time.sleep(5)

print "Code is shutting down. Goodbye."
