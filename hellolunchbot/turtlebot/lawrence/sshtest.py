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
    global shut_down_robot
    if button == "B0" and state == "pressed":
	robot_is_ready = True
        rospy.loginfo("Robot is ready for the next destination.")
    elif button == "B2" and state == "pressed":
        rospy.loginfo("B2 pressed, shutting down code.")
        shut_down_robot = True


rospy.init_node("ssh_node")
robot_is_ready = True  # Initial value for statement.

# Run button_event_callback when button is pressed:
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_event_callback)

robot_is_ready = False
shut_down_robot = False

print "To send to next destination: press b0 on the robot base."
print "To exit: press either b2 on the robot (prefered method) or ctrl+shift+\ "

while(not shut_down_robot):
    #if robot is ready
    if(robot_is_ready):
        print 'enter ssh'
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy()) # this will automatically add the keys
        ssh.connect("tavernaweb", username="kjin", password="lunchbot")

        print 'sqlsecond below'
        stdin, stdout, stderr = ssh.exec_command('python sqlsecond.py')
        yolo = stdout.readlines()  #room name
        print yolo[0]
        time.sleep(2)
        # example 2 : change ip address
        print 'changing ip address'
        stdin, stdout, stderr = ssh.exec_command('sed -i -- s/'+'132.178.226.152'+'/'+'10.254.38.45'+'/g /etc/sysconfig/network-scripts/ifcfg-eth0')
        print stdout.readlines()
        time.sleep(2)
        if yolo[0] != "dock":
            robot_is_ready = False

        os.system("python go_to_v1.py " + yolo[0])
        time.sleep(5)

        print "To send to next destination: press b0 on the robot base."
        print "To exit: press either b2 on the robot (prefered method) or ctrl+shift+\ "

    else:
        print "waiting for button press."
        time.sleep(5)

print "Code is shutting down. Goodbye."
