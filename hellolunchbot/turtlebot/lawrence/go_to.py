"""
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, self, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

This code was modified by Lawrence Kimsey, Mechanical Engineering Student at Boise State in Fall of 2016, using code
written by Mark Silliman, as mentioned above. The purpose of this code is to tell a Turtlebot robot how to move to a
requested location on a map. The map used in this project is engineering.yaml, and the following nodes should be started
in command prompt on the netbook while docked before running this code:
>roslaunch turtlebot_bringup minimal.launch
>roslaunch turtlebot_navigation amcl_bsu.launch
>roslaunch kobuki_auto_docking minimal.launch --screen

This code needs to be run with two arguments to work, for example, to go to room ENGR201C from MEC202C you could type:
> python go_to.py engr201c mec202
The first argument is the destination. Check the library below for a full list. You can't miss it. The second argument
is where the robot starts, with current options being mec202, engr201, engr240, and hallway. This helps the robot know
which doors it has to pass through and set intermediate waypoints.

"""

# TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, \
    SensorState  # for kobuki base power and auto docking
import sys
import time


class GoToPose:
    def move_to_goal(self, room):
        rospy.init_node('navigation', anonymous=False)

        # what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        # tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        # allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        # set goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pick_location(room)

        # start moving
        self.move_base.send_goal(goal)
        rospy.loginfo("Moving...")

        # allow TurtleBot up to 300 seconds (5 minutes) to complete task
        success = self.move_base.wait_for_result(rospy.Duration(300))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("The robot has reached the waypoint: " + room)
                # choose location based on input argument

    # the following is a list of available arguments and their corresponding coordinates:
    @staticmethod
    def pick_location(room):
        switcher = {
            # non-room waypoints:
            "dock": Pose(Point(-5.28, -4.40, 0.000), Quaternion(0.000, 0.000, 1.000, -0.014)),
            "door1in": Pose(Point(-7.904, 18.573, 0.000), Quaternion(0.000, 0.000, 0.710, 0.705)),
            "door1out": Pose(Point(-7.56, 20.5, 0.000), Quaternion(0.000, 0.000, -0.704, 0.710)),
            "door2in": Pose(Point(17.187, 88.307, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707)),
            "door2out": Pose(Point(17.098, 85.857, 0.000), Quaternion(0.000, 0.000, 0.701, 0.713)),
            "door3in": Pose(Point(-8.646, 87.633, 0.000), Quaternion(0.000, 0.000, 0.648, 0.762)),
            "door3out": Pose(Point(-8.853, 85.408, 0.000), Quaternion(0.000, 0.000, 0.741, 0.672)),
            "test": Pose(Point(-9.69, 85.1, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            # MEC202 rooms:
            "mec202a": Pose(Point(-8.11, 18.3, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202b": Pose(Point(-8.16, 16.2, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202c": Pose(Point(-8.10, 11.3, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202d": Pose(Point(-7.97, 8.96, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202e": Pose(Point(-7.96, 4.48, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202f": Pose(Point(-7.91, 2.37, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202g": Pose(Point(-7.82, -2.01, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202h": Pose(Point(-7.74, -4.07, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202j": Pose(Point(-7.49, -7.41, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202k": Pose(Point(-5.32, -8.15, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202l": Pose(Point(-1.84, -8.00, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202m": Pose(Point(.351, -7.88, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202n": Pose(Point(2.76, -7.72, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202p": Pose(Point(.641, -7.00, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202r": Pose(Point(-1.94, 2.89, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202s": Pose(Point(-3.16, 13.7, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202t": Pose(Point(-6.97, 16.9, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "mec202u": Pose(Point(-2.65, 19.3, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            # MEC202 cubicles:
            "cubicle1": Pose(Point(-1.49, 12.9, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle2": Pose(Point(-1.50, 12.1, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle3": Pose(Point(-1.30, 10.7, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle4": Pose(Point(-1.05, 8.71, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle5": Pose(Point(-.889, 7.27, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle6": Pose(Point(-.610, 5.76, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle7": Pose(Point(-.805, 3.92, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle8": Pose(Point(-4.20, 12.0, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle9": Pose(Point(-5.78, 12.1, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle10": Pose(Point(-2.97, 11.5, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle11": Pose(Point(-4.44, 11.2, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle12": Pose(Point(-5.99, 11.1, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle13": Pose(Point(-3.02, 7.85, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle14": Pose(Point(-4.37, 7.91, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle15": Pose(Point(-5.96, 7.96, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle16": Pose(Point(-2.88, 7.02, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle17": Pose(Point(-4.20, 6.87, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle18": Pose(Point(-5.87, 6.82, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle19": Pose(Point(-2.56, 3.97, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle20": Pose(Point(-4.16, 3.64, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle21": Pose(Point(-5.67, 3.66, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle22": Pose(Point(-2.14, 2.76, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle23": Pose(Point(-4.20, 2.58, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle24": Pose(Point(-5.65, 2.50, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle25": Pose(Point(-2.37, -.312, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle26": Pose(Point(-4.02, -.226, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            "cubicle27": Pose(Point(-5.67, -.229, 0.000), Quaternion(0.000, 0.000, 0.892, -1.500)),
            # ENGR building rooms:
            "engr203": Pose(Point(18.024, 82.713, 0.000), Quaternion(0.000, 0.000, 0.994, -0.112)),
            "engr204": Pose(Point(18.163, 75.192, 0.000), Quaternion(0.000, 0.000, 1.000, 0.010)),
            "engr205": Pose(Point(18.064, 73.617, 0.000), Quaternion(0.000, 0.000, 1.000, 0.015)),
            "engr206": Pose(Point(17.921, 66.357, 0.000), Quaternion(0.000, 0.000, 1.000, 0.025)),
            "engr207": Pose(Point(17.739, 64.554, 0.000), Quaternion(0.000, 0.000, 1.000, 0.016)),
            "engr208": Pose(Point(17.807, 60.144, 0.000), Quaternion(0.000, 0.000, 0.999, 0.044)),
            "engr209": Pose(Point(17.685, 58.171, 0.000), Quaternion(0.000, 0.000, 1.000, -0.012)),
            "engr212": Pose(Point(15.959, 82.992, 0.000), Quaternion(0.000, 0.000, 0.092, 0.996)),
            "engr213": Pose(Point(16.068, 68.250, 0.000), Quaternion(0.000, 0.000, -0.008, 1.000)),
            "engr214": Pose(Point(15.762, 61.835, 0.000), Quaternion(0.000, 0.000, -0.014, 1.000)),
            "engr215": Pose(Point(15.947, 57.818, 0.000), Quaternion(0.000, 0.000, -0.106, 0.994)),
            "engr221": Pose(Point(10.784, 47.722, 0.000), Quaternion(0.000, 0.000, 0.722, 0.692)),
            "engr222": Pose(Point(6.801, 47.905, 0.000), Quaternion(0.000, 0.000, 0.720, 0.693)),
            "engr223": Pose(Point(5.003, 47.825, 0.000), Quaternion(0.000, 0.000, 0.719, 0.695)),
            "engr224": Pose(Point(0.803, 47.787, 0.000), Quaternion(0.000, 0.000, 0.718, 0.696)),
            "engr225": Pose(Point(-1.146, 47.815, 0.000), Quaternion(0.000, 0.000, 0.712, 0.702)),
            "engr227": Pose(Point(7.158, 47.794, 0.000), Quaternion(0.000, 0.000, 0.675, 0.738)),
            "engr229": Pose(Point(-8.209, 56.890, 0.000), Quaternion(0.000, 0.000, 0.023, 1.000)),
            "engr230": Pose(Point(-8.469, 62.962, 0.000), Quaternion(0.000, 0.000, 0.020, 1.000)),
            "engr231": Pose(Point(-8.553, 64.806, 0.000), Quaternion(0.000, 0.000, 0.047, 1.000)),
            "engr232": Pose(Point(-8.832, 70.328, 0.000), Quaternion(0.000, 0.000, -0.026, 1.000)),
            "engr233": Pose(Point(-8.891, 72.620, 0.000), Quaternion(0.000, 0.000, 0.030, 1.000)),
            "engr234": Pose(Point(-9.171, 78.016, 0.000), Quaternion(0.000, 0.000, 0.026, 1.000)),
            "engr235": Pose(Point(-9.217, 80.138, 0.000), Quaternion(0.000, 0.000, 0.036, 1.000)),
            "engr236": Pose(Point(-9.348, 85.259, 0.000), Quaternion(0.000, 0.000, 0.018, 1.000)),
            "engr238": Pose(Point(-6.526, 60.187, 0.000), Quaternion(0.000, 0.000, 0.992, -0.112)),
            "engr239": Pose(Point(-7.704, 82.611, 0.000), Quaternion(0.000, 0.000, 0.996, 0.095)),
            # ENGR201 rooms:
            "engr201a": Pose(Point(13.839, 94.678, 0.000), Quaternion(0.000, 0.000, 0.011, 1.000)),
            "engr201b": Pose(Point(17.741, 95.502, 0.000), Quaternion(0.000, 0.000, -0.710, 0.704)),
            "engr201c": Pose(Point(17.838, 93.607, 0.000), Quaternion(0.000, 0.000, 0.000, 3.111)),
            "engr201-1": Pose(Point(14.600, 89.892, 0.000), Quaternion(0.000, 0.000, 0.019, 1.000)),
            "engr201-2": Pose(Point(16.543, 91.396, 0.000), Quaternion(0.000, 0.000, 0.724, 0.382)),
            # ENGR240 rooms: 
            "engr240a": Pose(Point(-10.300, 92.366, 0.000), Quaternion(0.000, 0.000, 0.023, 1.000)),
            "engr240b": Pose(Point(-10.447, 93.622, 0.000), Quaternion(0.000, 0.000, 0.007, 1.000)),
            "engr240c": Pose(Point(-10.022, 94.902, 0.000), Quaternion(0.000, 0.000, -0.656, 0.755)),
            "engr240-1": Pose(Point(-6.624, 88.356, 0.000), Quaternion(0.000, 0.000, 0.724, 0.690)),
            "engr240-2": Pose(Point(-9.379, 91.106, 0.000), Quaternion(0.000, 0.000, 0.575, 0.818)),
            "engr240-3": Pose(Point(-7.633, 92.686, 0.000), Quaternion(0.000, 0.000, 1.000, 0.028)),
        }
        return switcher.get(room, "error")

    @staticmethod
    def shutdown():
        rospy.loginfo("Stop")


class PlanRoute:
    def __init__(self):
        move = GoToPose()

        # turn arguments into lowered strings
        room_name = str(sys.argv[1]).lower()  # Where the robot will go!
        start_location = str(sys.argv[2]).lower()  # Where the robot starts!

        # Check for incorrect argument 1.
        if move.pick_location(room_name) == "error":
            print("The first argument is not correct! Please check that you typed your destination correctly.")
            return

        # Check for incorrect argument 2.
        if not (start_location == "mec202" or start_location == "engr201" or start_location == "engr240" or start_location == "hallway"):
            print("The second argument is not correct! Possible arguments: mec202, engr201, engr240, hallway")
            return

        # is the room inside the MEC202 offices?
        if room_name[:6] == "mec202" or room_name == "dock" or room_name[:7] == "cubicle":
            # Does it need to go through any doors?
            if start_location == "engr201":
                move.move_to_goal("door2in")  # Move to ENGR201 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!
				1) make sure that the door is closed (check turtlebot velocity, !success, kinect) 
				2) begin the audio tracks
				3) keep checking to see whether the door has been opened
				4) end the audio tracks
				5) "move.move_to_goal(room_name)] ###
            elif start_location == "engr240":
                move.move_to_goal("door3in")  # Move to ENG240 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            if start_location != "mec202":
                move.move_to_goal("door1out")  # Move to MEC202 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            move.move_to_goal(room_name)  # Go to destination.

        # Is the room inside the ENGR201 offices?
        elif room_name[:7] == "engr201":
            # Does it need to go through any doors?
            if start_location == "mec202":
                move.move_to_goal("door1in")  # Move to MEC202 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###
            elif start_location == "engr240":
                move.move_to_goal("door3in")  # Move to ENGR240 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            if start_location != "engr201":
                move.move_to_goal("door2out")  # Move to ENGR201 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            move.move_to_goal(room_name)  # Go to destination.

        # Is the room inside the ENGR240 offices?
        elif room_name[:7] == "engr240":
            # Does it need to go through any doors?
            if start_location == "mec202":
                move.move_to_goal("door1in")  # Move to MEC202 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###
            elif start_location == "engr201":
                move.move_to_goal("door2in")  # Move to ENGR201 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            if start_location != "engr240":
                move.move_to_goal("door3out")  # Move to ENGR240 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            move.move_to_goal(room_name)  # Go to destination.

        # Is the room attached to the main hallway?
        else:
            # Does it need to go through any doors?
            if start_location == "mec202":
                move.move_to_goal("door1in")  # Move to MEC202 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###
            elif start_location == "engr201":
                move.move_to_goal("door2in")  # Move to ENGR201 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###
            elif start_location == "engr240":
                move.move_to_goal("door3in")  # Move to ENGR240 door.
                ### [INSERT FUTURE DOOR INTERACTION HERE!] ###

            move.move_to_goal(room_name)  # Go to destination.

        if room_name == "dock":  # Start docking procedure if destination is dock
            self.dock_with_base()

    def dock_with_base(self):
        self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
        rospy.loginfo("waiting for auto_docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo(
            "Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
        self._client.send_goal(goal)
        time.sleep(30)  # Give robot 30 seconds to dock. 


if __name__ == '__main__':
    try:
        PlanRoute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

