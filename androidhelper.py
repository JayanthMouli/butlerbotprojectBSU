import MySQLdb
from urllib2 import urlopen
import time
import json
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import os
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, \
    SensorState  # for kobuki base power and auto docking
import sys
from go_to import GoToPose
from Speak import Speak
from undock import GoForward
from go_to_3 import GoToPose
def findNumRows() :
	openUrlinit = urlopen('https://butlerbot.000webhostapp.com/checkforupdate.php')
	init= int(openUrlinit.read())
	
	try :
		while True:
			openUrl = urlopen('https://butlerbot.000webhostapp.com/checkforupdate.php')
			val = int(openUrl.read())
			time.sleep( 2 )
			if val>init :
				break
			
	except Exception as e:
		print(e)
	finalurl = urlopen('https://butlerbot.000webhostapp.com/returnval.php')
	finalstring = finalurl.read().lower()
	a = Speak("going to " + finalstring)
	b = GoForward()
	os.system("python go_to_3.py " + finalstring + " mec202")	

if __name__ == '__main__':
	findNumRows()
