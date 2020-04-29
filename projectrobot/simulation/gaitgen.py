#!/usr/bin/env python
import rospy
import std_msgs.msg
from xbot_msgs.msg import JointCommand
import time
import math
import csv
import numpy as np
import matplotlib.pyplot as plt

#Gait Generator
#Log XYZ Position of foot w/ respect to time
#Use inverse kinematics to check if trajectory possible or use workspace chart for valid positions.
#Assume standing is 100mm height.
# x = 0.5y when walking. ie 50mm forward, 25mm above ground when trajectory.
#Function for step size, changable. Max step size = max y above ground.
#Static Gait first, one leg at a time.
#When turning, make Z on separate graph, with +ve and -ve as left right etc.
#Computation order: FL moves, FR moves, RL moves, RR moves. Should be able to do front and rear
# at the same time. If same time, don't need to shift the torso forward. If completely static,
# torso should move after front is moved. EE -x = shifting the torso forward. 
# FL +xy, FR +xy, #All -x, RL +xy, RR +xy, repeat.
# Do forward, back first.
 
#Constants, Don't change unless leg overhaul
#mm
horiz = 25.0 #From motor1 rotation point to that horizontal link 
thigh1 = 60.0 #Longer link that connects from motor to calf
thigh2 = 50.0 #Shoter link that connects from that horizontal link to calf
thighgap = 20.0 #Knee joint to the thigh2 joint
calflink = 70.0 #Calf link


T = 1.0 #Time period for gait, ie 4 leg movements or one step

stepsize = 51 #mm x distance of step
yratio = 2 #Semi-circle for simplicities sake. Height of step ratio to radius of step.

#Maybe make height static value, smaller steps will be tiny
#Temp Inputs Until User Control made

commandin = ["Forward", stepsize, yratio, T]
#Command = [Direction, Step Size, Step height, Step period]
shiftarray = [0]*stepsize
trajarray = [0]*stepsize
zeroarray = [0]*stepsize
ydistance = [0]*stepsize
yrelative = [0]*stepsize
anglearray = [0]*stepsize
shiftangles = [0]*stepsize
#def controlinput():

#def stepsizegen(): 
	#return



def movement():

	global xpos
	global ypos
	global zpos
	xpos = 0
	ypos = -100

	arrays = gaitgen() #Get the array bundle, Will make the commandin input later, keep constant

	shift = arrays[0]
	traj = arrays[1]
	z = arrays[2]

	#xyzcommand = [["FL"],["FR"],["RL"],["RR"]] #Just make 4 different arrays
	#xyzcommand.insert(1,traj)

	phase = 3 #5 phases, FL,FR,SHIFT,RL,RR


	#Set phase -> pull data out of array[0] then delete and shift until empty
	#1 Leg at a time or the invkinematics will lag out the script
	#Keep as xyz now, easy to display and debug. 
	#For loop later to convert traj and shift into motor angles.
	#Don't convert real time maybe? Bad for control if not real time.
	#xyz -> leg assignment -> inverse -> motors?? Fast enough??
	#Preprocessing all angles is inefficient for control
	#
	#Zero arrays are pretty useless, motors are doing nothing anyway
	#Needed for data???



	j=0
	#print traj[0][1] #Row, X Y Z
	if not phase == 3:
		while(j < stepsize):
			x = traj[j][0]
			y = traj[j][1]
			anglearray[j] = inversekinematics(x,y)
			j = j + 1
	if phase == 3:
		while(j < stepsize):
			x = shift[j][0]
			y = shift[j][1]
			shiftangles[j] = inversekinematics(x,y)	
			j = j + 1
	#print anglearray
	print shift
	print shiftangles
	#Make another anglearray for left side motors before sending
	if phase == 1 or phase == 2:
		senddata(anglearray)
	if phase == 4 or phase == 5:
		print "no"
	if phase == 3:
		senddata(shiftangles)

	print "done"

	#2 motors each side
	#motors on this side = fine
	#motors on other side = angle limits are flipped
	#hard to verify with testing
	#can just apply an offset if motors are the ones on the left
	#ok

	#print xyzcommand [1][0]

	#print ydistance, len(ydistance)
	#print 5 % 2 == 0



	#print shiftarray
	#print trajarray
	#print zeroarray

	#Interpolate for every 18deg
	#Create two trajectory
	#One for forward step curve
	#One for body shift -x
	
	
	#stepxyzcommand = [[x,y,z],[x,y,z],[x,y,z]]
	#One stepxyzcommand array will be for one leg at a time.
	#Will need to do 4 leg command at the same time for body shift though.
	#ok, stepxyzcommand will be [4][[3]]

	#Multithread
	#plt.axis([0, 200, -100, 200]) # xlower,xhigher,ylower,yhigher #x = ms???, y = mm
	print traj
	print "---"
	print ydistance
	print "---"
	print yrelative
	a = 0
	#plt.axis([0, stepsize, 0, 60])
	plt.axis([0, stepsize, -15, 15])
	for i in range(stepsize):
		#plt.figure(1)
		#plt.subplot(211)
		#plt.scatter(anglearray[i][0], anglearray[i][1])
		#plt.subplot(212)
		#plt.scatter(i, ydistance[i])
		
		a =traj[i][1]
		#if i < stepsize:
			#a = yrelative[i]
		#if i >= stepsize:
		#	a = -yrelative[i]

		print a
		plt.xlabel("X displacement (mm)", fontdict=None, labelpad=None)
		plt.ylabel("Y displacement (mm)", fontdict=None, labelpad=None)
		plt.grid(b=True, which='major', axis='both')
		plt.scatter(i, a)
		plt.pause(0.0000001)

	plt.show() #Retains window after all functions have stopped







def senddata(arrayofangles):

	print "er"
	while(len(arrayofangles)>0):
		a = arrayofangles[0][0]
		b = arrayofangles[0][1]
		print a,b, len(arrayofangles)
		del arrayofangles[0]
		time.sleep(T/stepsize)






def gaitgen():
	#Input: Direction, Magnitude
	# Input Array

	#a = stepsizegen()
	#print a

	print commandin
	#stepsize = commandin[1]
	
	#Generate Trajectory
	#radius = stepsize*yratio #Step should be +xy #Interpolate three points
	i = 0
	switch = 0
	while (i < stepsize): #< because array starts at 0, if 50 then array is 0-49
		#print i
		#180/stepsize(50) = 3.6 deg per mm
		#might need to ceiling these numbers.
		#0 at start of the y array might be a problem.
		if (i < stepsize/2):
			#ydistance[i] = (abs(stepsize/2-i))*math.tan(math.radians(i*180.0/stepsize))
			ydistance[i] = yratio*(stepsize/2)*math.sin(math.radians(i*180.0/stepsize+1)) #stepsize+1 gets rid of the 1, -100 foot drag. need some y movement on first index.
		if (i == stepsize/2 and not stepsize % 2 == 0): #not even
			ydistance[i] = yratio*(stepsize/2) #Need brackets
			#print "Even"
			switch = 0
		elif (i == stepsize/2 and stepsize % 2 == 0): #even
			ydistance[i] = ydistance[i-1]
			#print "Odd"
			switch = 1
		if (i > stepsize/2 and switch == 0):
			ydistance[i] = ydistance[stepsize/2-int(abs(stepsize/2-i))]

		if (i > stepsize/2 and switch == 1):
			ydistance[i] = ydistance[stepsize/2-int(abs(stepsize/2-i))-1]

		#print ydistance,abs(stepsize/2-i)
		#get rid of index 25 0-24 = 25 25-50 = 25
		#if stepsize / 2 = odd
		#if (stepsize % 2 == 0 and i == stepsize-1): #if even do this
		#else: #if even do this
		#print i*180.0/stepsize, abs(stepsize/2-i), ydistance[i]
		#yrelative[i] = ydistance [i] - ydistance[i-1]
		#shiftarray[i] = [-stepsize/stepsize,0,0] #if step size is while limiter
		#trajarray[i] = [stepsize/stepsize,ydistance[i],0]
		#trajarray[i] = [stepsize/stepsize,yrelative[i],0]
		#zeroarray[i] = [0,0,0]
		i = i + 1
	
	del ydistance[stepsize/2]
	ydistance.append(0)
	i = 0

	while (i < stepsize):
		yrelative[i] = ydistance [i] - ydistance[i-1]
		shiftarray[i] = [-stepsize/stepsize,0,0] #if step size is while limiter
		#trajarray[i] = [stepsize/stepsize,ydistance[i],0]
		trajarray[i] = [stepsize/stepsize,yrelative[i],0]
		zeroarray[i] = [0,0,0]
		i = i + 1

	arraybundle = [shiftarray,trajarray,zeroarray]
	
	#print yrelative

	return arraybundle


	
def inversekinematics(x,y): #Need to know where the robot thinks its feet are.
	global xpos
	global ypos
	xpos = xpos + x
	ypos = ypos + y

	#print xpos, ypos

	inputx = xpos
	inputy = ypos

	origintoee = math.sqrt(inputx**2+inputy**2)

	kneeangle = math.acos( (thigh1**2.0 + calflink**2.0 - origintoee**2.0)/(2.0*thigh1*calflink) )

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


	motors = [motor1,motor2]

	return motors


def publisher():
	#rospy.init_node('JointPublisher', anonymous=True)
	movement()
	#rospy.spin()


if __name__ == '__main__':
	publisher()
