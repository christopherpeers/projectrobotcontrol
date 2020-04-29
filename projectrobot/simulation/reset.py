#!/usr/bin/env python
import rospy
import std_msgs.msg
from xbot_msgs.msg import JointCommand
import time

def angle():

	pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10)
	time.sleep(0.5)
	
	command = JointCommand()
	command.header.seq = 0
	command.header.stamp = rospy.Time.now()
	command.header.frame_id = ''

	#calf lower="-2.77507351067" upper="-0.610865238198"
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

	command.position = [0,1.57,-3.14, #FL
				0,1.57,-3.14, #FR
				0,0,1.57,-3.14, #RL
				0,0,1.57,-3.14, #RR
				0,0,0,0] #Feet
	#1.57 = 90deg
	command.velocity = [5]*18
	command.effort = [5]*18
	command.stiffness = [0]*18
	command.damping = [0]*18
	command.ctrl_mode = [1]*18
	command.aux_name = ''
	command.aux = [0]*18
	print command.name
	print command.position
	time.sleep(0.1)
	pub.publish(command)

	time.sleep(3)

	command.position = [0,-0.20135792079,-2.364806627, #FL
				0,-0.20135792079,-2.364806627, #FR
				0,0,-0.20135792079,-2.364806627, #RL
				0,0,-0.20135792079,-2.364806627, #RR
				1.13,1.13,1.13,1.13] #Feet

	pub.publish(command)
	time.sleep(3)

	command.position = [0,-0.811034394288,-1.389669322, #FL
				0,-0.811034394288,-1.389669322, #FR
				0,0,-0.744537254074,-1.50666291141, #RL
				0,0,-0.744537254074,-1.50666291141, #RR
				0.632296369706,0.632296369706,0.682792819072,0.682792819072] #Feet

	pub.publish(command)

	quit()


def publisher():
	rospy.init_node('JointPublisher', anonymous=True)
	angle()
	rospy.spin()


if __name__ == '__main__':
	publisher()
