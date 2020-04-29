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



def kinematics():
######################################################################################
#Inverse Kinematics
		print "###########################################"

		inputx = -30
		inputy = -70
		newdata = [0,0,0]
		data = [0,0,0]

		while inputx != 31:


			origintoee = math.sqrt(inputx**2+inputy**2)


			kneeangle = math.acos( (thigh1**2.0 + calflink**2.0 - origintoee**2.0)/(2.0*thigh1*calflink) )

			#if sign
			motortoeeangle = math.atan2(inputy,inputx) + math.radians(180)

			thigh1toeeangle = math.acos( (thigh1**2.0 + origintoee**2.0 - calflink**2.0)/(2.0*thigh1*origintoee) )

			motor1 = motortoeeangle - thigh1toeeangle
	################################################



			a = math.sqrt(thigh1**2.0 + thighgap**2.0 - 2.0*thigh1*thighgap*math.cos(kneeangle))



			b = math.acos( (a**2.0 + horiz**2.0 - thigh2**2.0)/(2.0*a*horiz) )



			c = math.acos( (a**2.0 + thigh1**2.0 - thighgap**2.0)/(2.0*a*thigh1) )



	################################################
			motor2 = motor1 + b + c

			motor1 = math.degrees(motor1)
			motor2 = math.degrees(motor2)

			if motor1 > 180.0 or motor2 > 180.0:
				motor1 = motor1 - 360.0
				motor2 = motor2 - 360.0

			print origintoee
			print math.degrees(kneeangle)
			print math.degrees(motortoeeangle)
			print math.degrees(thigh1toeeangle)
			print (thigh1**2.0 + origintoee**2.0 - calflink**2.0), (2.0*thigh1*origintoee)

			print a
			print math.degrees(b)
			print math.degrees(c)
			print -math.radians(motor1), -(3.14-kneeangle)
			print "this", inputx, inputy, motor1, motor2
			#30,68

			footangle = -(1.57-math.radians(360 -(180-motor1 + math.degrees(kneeangle) + 90) + 90))
			print footangle, -(1.57-math.radians(footangle))

			print "###########################################"

			newdata = [inputx, -(math.radians(motor1)-1.57), -(math.radians(motor2)-1.57)]
			data = np.vstack([data, newdata])
				
			inputx = inputx + 1












		end = 1
		#print data
		#CHANGE NAME OF CSV FILE HERE
		np.savetxt(".csv", data, delimiter=",")
		#np.savetxt("data2.csv", debug, delimiter=",")
		quit()


def publisher():
	#rospy.init_node('JointPublisher', anonymous=True)
	kinematics()
	#rospy.spin()


if __name__ == '__main__':
	publisher()
