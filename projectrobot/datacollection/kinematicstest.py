#!/usr/bin/env python
import rospy
import std_msgs.msg
from xbot_msgs.msg import JointCommand
import time
import math
import csv
import numpy as np


#Angles are +ve from anticlockwise horizontal
#deg 0-180, only lower half of motors used
#motorangle1 = 50.0 #deg
#motorangle2 = 20.0 #deg

#Constants, Don't change unless leg overhaul
#mm
horiz = 25.0 #From motor1 rotation point to that horizontal link 
thigh1 = 60.0 #Longer link that connects from motor to calf
thigh2 = 50.0 #Shoter link that connects from that horizontal link to calf
thighgap = 20.0 #Knee joint to the thigh2 joint
calflink = 70.0 #Calf link

#Origin:- (0,0) on Motor 1 axis of rotation
origin = [0.0,0.0]
kneex = 0.0
kneey = 0.0
footx = 0.0
footy = 0.0

#Equation of circle
#(x-h)**2 + (y-k)**2 = r**2

#Sintheta = O/H
#math.acos( (y**2 + x**2 - z**2) / (2*y*x) )
#All sides = Angle
#Side = sqrt( b**2 + c**2 -2bc math.cos(A) )


#NEW LIMITS
#MOTOR1
#-45 to 180
#MOTOR2
#30 to 150

lowerlimit1 = -45.0
upperlimit1 = 180.0

lowerlimit2 = 30.0
upperlimit2 = 150.0


def kinematics():
	#if motorangle1 > motorangle2:

	motorangle1 = -45.0 #deg
	motorangle2 = 30.0 #deg
	n = 0 #error counter
	data = [0,0,0,0]
	debug = [0,0,0,0,0,0,0,0,0]

	while(motorangle1 <= upperlimit1): #Changed to 1 from 180 to make testing faster

		print motorangle1, motorangle2

		angle1 = abs(motorangle1-motorangle2)
		#angle1 is difference
		angle1 = math.radians(angle1)

		side1 = math.sqrt(thigh1**2.0 + horiz**2.0 - 2.0*thigh1*horiz*math.cos(angle1))

		print "er", (thigh1**2.0 + side1**2.0 - horiz**2.0)/(2.0*thigh1*side1)

		angle2 = math.acos( (thigh1**2.0 + side1**2.0 - horiz**2.0)/(2.0*thigh1*side1) )
	
		print "error check", (thighgap**2.0 + side1**2.0 - thigh2**2.0)/(2.0*thighgap*side1)

		a = (thighgap**2.0 + side1**2.0 - thigh2**2.0)/(2.0*thighgap*side1)




		print a
		#IF INVALID
		if ((thighgap**2.0 + side1**2.0 - thigh2**2.0)/(2.0*thighgap*side1)) > 1:
			print "pass"
			n = n+1
			
			newdata = [motorangle1, motorangle2, 0, 0]
			data = np.vstack([data, newdata])


			if motorangle2 >= upperlimit2:
				motorangle1 = motorangle1 + 1.0
				motorangle2 = lowerlimit2
			motorangle2 = motorangle2 + 1.0
			continue		



		#print (thighgap**2.0 + side1**2.0 - thigh2**2.0)
		#print (2.0*thighgap*side1)
		print side1, math.degrees(angle1), math.degrees(angle2)

		angle3 = math.acos(a)




		#If angle3 is negative then you get the wrong solution
		angles = angle2 + angle3
	
		#print angle1
		#print thigh1**2.0 + horiz**2.0 - 2.0*thigh1*horiz*math.cos(angle1)
		#print angle2
		#print math.degrees(angle2)
		#print math.degrees(angle3)
		#print math.degrees(angles)

		if angle3 < 0:
			print "no"
			quit()

	#----------------------------------------------------------------------------------------
	#Forward Solution
	#Up Right +ve



		#Knee
		if motorangle1 > 90:
			kneex = -math.cos(math.radians(motorangle1))*thigh1
			kneey = -math.sin(math.radians(motorangle1))*thigh1
		if motorangle1 < 90 and motorangle1 > 0:
			kneex = math.sin(math.radians(motorangle1)- math.radians(90))*thigh1
			kneey = -math.cos(math.radians(motorangle1)- math.radians(90))*thigh1
		if motorangle1 == 90:
			#kneex = -math.cos(math.radians(motorangle1))*thigh1
			kneey = -thigh1
		if motorangle1 == 0:
			#kneex = -math.cos(math.radians(motorangle1))*thigh1
			kneex = -thigh1
		if motorangle1 < 0:
			kneex = -math.cos(math.radians(abs(motorangle1)))*thigh1
			kneey = math.sin(math.radians(abs(motorangle1)))*thigh1

		#Calf

		#X Y OF FOOT FLIPS AT 66-67

		#angle2 and 3 already in rads
		angle4 = angles - math.radians(motorangle1)
		#if angle4 < 0:
		#	print "angle4 negative", angle4
		#print math.degrees(angle4), "angle4"
	
		if math.degrees(angle4) < 90:
			footx = math.cos(angle4)*calflink
			footy = -math.sin(angle4)*calflink
		if math.degrees(angle4) > 90:
			#angle4 = angle4 - math.radians(90)
			footx = -math.sin(angle4- math.radians(90))*calflink
			footy = -math.cos(angle4- math.radians(90))*calflink
		if math.degrees(angle4) == 90:
			#footx = math.cos(math.radians(angle4))*calflink
			footy = -calflink

		#angle2+3 - motorangle1 = angle from horizontal to leg

		#motorangle1 = math.radians(motorangle1)
		#motorangle2 = math.radians(motorangle2)
		#if motorangle1 < 90:
			#o1 = math.asin(motorangle1)*thighlink1
			#a1 = 

		#if motorangle1 < 90:
		#if motorangle1 == 90:
	
	

		#print kneex, kneey, footx, footy
		#IF ALL IS FINE

		newdata = [motorangle1, motorangle2, kneex + footx, kneey + footy]
		data = np.vstack([data, newdata])
		print newdata
		
		debugdata = [motorangle1, motorangle2, kneex, kneey, footx, footy, math.degrees(angle4) , math.degrees(angles), motorangle1]
		debug = np.vstack([debug, debugdata])


		if motorangle2 >= upperlimit2:
			motorangle1 = motorangle1 + 1.0
			motorangle2 = lowerlimit2
		motorangle2 = motorangle2 + 1.0
		print "xy = ", kneex + footx, kneey + footy

	print "no error", n
	#print data
	#CHANGE NAME OF CSV FILE HERE
	np.savetxt("kinematicsdatatest.csv", data, delimiter=",")
	#np.savetxt("data2.csv", debug, delimiter=",")
	quit()


def publisher():
	#rospy.init_node('JointPublisher', anonymous=True)
	kinematics()
	#rospy.spin()


if __name__ == '__main__':
	publisher()
