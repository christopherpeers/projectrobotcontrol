#!/usr/bin/env python
import rospy
import std_msgs.msg
from xbot_msgs.msg import JointCommand
import time
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from pynput.keyboard import Key, Listener
 
#Constants, Don't change unless leg overhaul
#mm
#chassiswidth = 
#chassislength =

horiz = 25.0 #From motor1 rotation point to that horizontal link 
thigh1 = 60.0 #Longer link that connects from motor to calf
thigh2 = 50.0 #Shoter link that connects from that horizontal link to calf
thighgap = 20.0 #Knee joint to the thigh2 joint
calflink = 70.0 #Calf link


T = 1.0 #Time period for gait, ie 4 leg movements or one step

stepsize = 50 #mm x distance of step
yratio = 2.5 #Semi-circle for simplicities sake. Height of step ratio to radius of step.


#commandin = ["Forward", stepsize, yratio, T]
#Command = [Direction, Step Size, Step height, Step period]
shiftarray = [0]*stepsize
trajarray = [0]*stepsize
zeroarray = [0]*stepsize
ydistance = [0]*stepsize
yrelative = [0]*stepsize
anglearrayfront = [0]*stepsize
anglearrayback = [0]*stepsize
shiftanglesfront = [0]*stepsize
shiftanglesback = [0]*stepsize


def on_release(key):
	global controldirection
	print "\n stay"
	controldirection = "Null"
	if key == Key.ctrl: #Ctrl+C
		#End Keyboard thing
		return False 
def check_key(key):
	global controldirection
	if key in [Key.up]:
		print('\n forward')
		controldirection = "Forward"
	if key in [Key.down]:
		print('\n backwards')
		controldirection = "Reverse"
	if key in [Key.left]:
		print('\n left')
		controldirection = "LeftRotation"
	if key in [Key.right]:
		print('\n right')
		controldirection = "RightRotation"

def startup():
###################################### INIT KEYBOARD CONTROL
	listener = Listener(
	    on_press=check_key,
	    on_release=on_release)
	listener.start()
###################################### INIT GLOBAL VARIABLES 
	global controldirection
	controldirection = "Null"
	global gaitselection
	gaitselection = "Default" #Choices are Default,
	

	global commandin
	commandin = [controldirection, stepsize, yratio, T, gaitselection]
	#Control direction should be 360 deg polar form.

###################################### START STAND UP, DETERMINE FEET POSITIONS
	#standup() m1 = 46.4688478326 m2 = 103.543928783 m3 = whatever is horizontal
	global xpos #mm
	global ypos #mm
	global zpos #mm
	xpos = 0
	ypos = -100
	zpos = 0
	print "Ready"
###################################### CONTROL PUBLISHER
	global pub
	pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10)
	time.sleep(0.5)
###################################### START CONTROL LOOP

	controlloop()

	#while(1):
		#print controldirection
	#	commandin = [controldirection, stepsize, yratio, T] #Listen to commandss
	#	print commandin
		#time.sleep(0.1)
		
def controlloop():
	global commandin
	global xpos #mm
	global ypos #mm
	n = 0
	while not rospy.is_shutdown():
		commandin = [controldirection, stepsize, yratio, T, gaitselection] #Listen to command changes

		#Generate trajectory for direction 

		#array = gaitgen(commandin) #Trajectory generation function

		################################Careless Coding Start
		if (n == 0):
			#get x y array
			xyarray = trajgenxy(commandin) #Get only once?
			shift = xyarray[0]
			traj = xyarray[1]
			n = 1

		#print traj

		#x y array to x y z array using direction

		#x y z array to angle array
		if (n == 1):
			j = 0
			while(j < stepsize):
				x = traj[j][0]
				y = traj[j][1]
				anglearrayfront[j] = inversekinematics(x,y)
				j = j + 1

			j = 0
			#resetf()
			while(j < stepsize):
				x = shift[j][0]
				y = shift[j][1]
				shiftanglesfront[j] = inversekinematics(x,y)
				j = j + 1

			j = 0
			#resetr()
			while(j < stepsize):
				x = shift[j][0]
				y = shift[j][1]
				shiftanglesback[j] = inversekinematics(x,y)	
				j = j + 1
			
			j = 0
			#resetr()
			while(j < stepsize):
				x = traj[j][0]
				y = traj[j][1]
				anglearrayback[j] = inversekinematics(x,y)
				j = j + 1

			#resetf()
			n = 2


		phase = 0

		if (gaitgen(commandin) == True):
			stepf = anglearrayfront[:]
			stepr = anglearrayback[:]
			shiftf = shiftanglesfront
			shiftr = shiftanglesback

			while(len(stepf)>0):
				sendfl(stepf)
				#print stepf
				print "FL"
				del stepf[0]
				time.sleep(0.1)

			time.sleep(4)

			stepf = anglearrayfront[:]

			while(len(stepf)>0):
				sendfr(stepf)
				#print stepf
				print "FR"
				del stepf[0]
				time.sleep(0.1)

			time.sleep(4)

			while(len(shiftf)>0):
				sendall(shiftf,shiftr)
				#print stepf
				print "Shift"
				del shiftf[0]
				del shiftr[0]
				time.sleep(0.1)

			time.sleep(4)

			while(len(stepr)>0):
				sendrl(stepr)
				#print stepf
				print "RL"
				del stepr[0]
				time.sleep(0.1)

			time.sleep(4)

			stepr = anglearrayback[:]

			while(len(stepr)>0):
				sendrr(stepr)
				#print stepf
				print "RR"
				del stepr[0]
				time.sleep(0.1)

		################################Careless Coding End


		time.sleep(T/10)



def resetf():
	global xpos #mm
	global ypos #mm
	xpos = 0
	ypos = -100

def resetr():
	global xpos #mm
	global ypos #mm
	xpos = 0
	ypos = -95

def gaitgen(command):
	#Base rotation is a velocity, polar is direction -> stepsize + time = velocity.	
	
	direction = command[0]


	if (direction == "Null"):
		print "stop"
		#If no control input, check foot positions, square up.
		#print "[", xpos,",", ypos,",", zpos, "]"
		return
	if (direction == "Forward"):
		polardirection = 0
		baserotation = 0
	if (direction == "Reverse"):
		polardirection = 180
		baserotation = 0
	#if (direction == "LeftStrafe"):
	#if (direction == "RightStrafe"):
	if (direction == "LeftRotation"): #Rotate only
		polardirection = "Null"
		baserotation = -30 #deg
	if (direction == "RightRotation"):
		polardirection = "Null"
		baserotation = 30 #deg

	#Calculate foot destination X,Z. Y will be the same

	stepradius = commandin[1] #mm
	#stepangle = baserotation

	if (not polardirection == "Null"):
		stepangle = baserotation
		x = stepradius*math.cos(math.radians(baserotation))
		z = stepradius*math.sin(math.radians(baserotation))


		#if (polardirection >= 0 and polardirection < 90): #1st

		if (polardirection >= 90 and polardirection < 180): #2nd
			x = -x

		if (polardirection >= 180 and polardirection < 270): #3rd
			x = -x
			z = -z

		if (polardirection >= 270 and polardirection < 360): #4th
			z = -z

		print x,z
		
		return True

	else:
		#Can't do static rotation until I have dimensions
		print "Work in Progress"		

		return "static rotation"



	#xyz per leg on motor rotation axis.

	#Once x,z calculated, send to gait gen then -> inverse kinematics.
	#Adjust gaitgen code to rotate the waveform by angle in y axis and make output 3D.

	#inversekinematics(x,y,z)

	#Start While Loop (5 iterations)
	#Then send, after sending, wait for T seconds.

def trajgenxy(command):
	#Generate Trajectory
	#radius = stepsize*yratio #Step should be +xy #Interpolate three points
	#commandin = [controldirection, stepsize, yratio, T, gaitselection]
	stepsize = command[1]
	yratio = command[2]
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
	return arraybundle


def inversekinematics(x,y): #Need to know where the robot thinks its feet are.
	global xpos
	global ypos
	#global zpos needs to be xyz later
	xpos = xpos + x
	ypos = ypos + y
	#zpos = zpos + z
	print xpos,ypos
	#print xpos, ypos

	inputx = xpos
	inputy = ypos

########################################################################### Don't touch start
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
########################################################################### Don't touch end

	#motors = [motor1,motor2]
	
	footangle = -(1.57-math.radians(360 -(180-motor1 + math.degrees(kneeangle) + 90) + 90))

	motors = [-math.radians(motor1),-(3.14-kneeangle),footangle] #thigh, calf, foot

	return motors

def sendall(x,y):
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	command.name = ['motorFL1_motorFL2', #0
			'motorFL2_thigh',    #1
			'FLthigh_calf',      #2
			'motorFR1_motorFR2', #3
			'motorFR2_thigh',    #4
			'FRthigh_calf',      #5
			'motorRL1_motorRL2', #6
			'motorRL2_motorRL3', #7
			'motorRL3_thigh',    #8
			'RLthigh_calf',      #9
			'motorRR1_motorRR2', #10
			'motorRR2_motorRR3', #11
			'motorRR3_thigh',    #12
			'RRthigh_calf',      #13
			'FLcalf_foot',       #14
			'FRcalf_foot',       #15
			'RLcalf_foot',       #16
			'RRcalf_foot']       #17

	command.position = [0,x[0][0],x[0][1], #FL
				0,x[0][0],x[0][1], #FR
				0,0,y[0][0],y[0][1], #RL
				0,0,y[0][0],y[0][1], #RR
				x[0][2],x[0][2],y[0][2],y[0][2]] #Feet

	#home = [0,-0.811034394288,-1.389669322,0,-0.811034394288,-1.389669322,0,0,-0.744537254074,-1.50666291141,0,0,-0.744537254074,-1.50666291141,0.632296369706,0.632296369706,0.682792819072,0.682792819072]

	command.velocity = [5]*18
	command.effort = [5]*18
	command.stiffness = [0]*18
	command.damping = [0]*18
	command.ctrl_mode = [1]*18
	command.aux_name = ''
	command.aux = [0]*18

	#time.sleep(0.1)

	pub.publish(command)

def sendfl(x):
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	command.name = ['motorFL1_motorFL2', 
			'motorFL2_thigh',    
			'FLthigh_calf',
			'FLcalf_foot']       

	command.position = [0,x[0][0],x[0][1],x[0][2]]

	command.velocity = [5]*4
	command.effort = [5]*4
	command.stiffness = [0]*4
	command.damping = [0]*4
	command.ctrl_mode = [1]*4
	command.aux_name = ''
	command.aux = [0]*4

	pub.publish(command)

def sendfr(x):
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	command.name = ['motorFR1_motorFR2', 
			'motorFR2_thigh',    
			'FRthigh_calf',
			'FRcalf_foot']       

	command.position = [0,x[0][0],x[0][1],x[0][2]]

	command.velocity = [5]*4
	command.effort = [5]*4
	command.stiffness = [0]*4
	command.damping = [0]*4
	command.ctrl_mode = [1]*4
	command.aux_name = ''
	command.aux = [0]*4

	pub.publish(command)

def sendrl(x):
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	command.name = ['motorRL1_motorRL2', 
			'motorRL2_motorRL3', 
			'motorRL3_thigh',    
			'RLthigh_calf',
			'RLcalf_foot']       

	command.position = [0,0,x[0][0],x[0][1],x[0][2]]

	command.velocity = [5]*5
	command.effort = [5]*5
	command.stiffness = [0]*5
	command.damping = [0]*5
	command.ctrl_mode = [1]*5
	command.aux_name = ''
	command.aux = [0]*5

	pub.publish(command)

def sendrr(x):
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	command.name = ['motorRR1_motorRR2', 
			'motorRR2_motorRR3', 
			'motorRR3_thigh',    
			'RRthigh_calf',
			'RRcalf_foot']       

	command.position = [0,0,x[0][0],x[0][1],x[0][2]]

	command.velocity = [5]*5
	command.effort = [5]*5
	command.stiffness = [0]*5
	command.damping = [0]*5
	command.ctrl_mode = [1]*5
	command.aux_name = ''
	command.aux = [0]*5

	pub.publish(command)

def publisher():
	rospy.init_node('JointPublisher', anonymous=True)
	startup()
	rospy.spin()


if __name__ == '__main__':
	publisher()
