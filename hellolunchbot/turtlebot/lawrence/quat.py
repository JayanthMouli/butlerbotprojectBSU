#!/usr/bin/env python

from tf import transformations
import math

#Enter value in degrees. 
turtlebot_orientation_in_degrees = 180
#convert euler to quaternion and save in new variable quat
quat = transformations.quaternion_from_euler(0, 0, math.radians(turtlebot_orientation_in_degrees))
#print current quat value
print quat
