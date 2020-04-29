#!/usr/bin/env python
import rospy
import std_msgs.msg
from xbot_msgs.msg import JointCommand
import time
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches




def gait():

	global flpos #mm
	global frpos #mm
	global rlpos #mm
	global rrpos #mm
	global chassispos

	#When x = 0 y = -100
	defaultflpos = [85,-114.5,0]
	defaultfrpos = [-85,-114.5,0]
	defaultrlpos = [120,53.5,0]
	defaultrrpos = [-120,53.5,0]
	defaultchassispos = [0,0,0]

	verts = [
		(defaultflpos[0],defaultflpos[1]),  # left, bottom
		(defaultfrpos[0],defaultfrpos[1]),  # left, top
		(defaultrlpos[0],defaultrlpos[1]),  # right, top
		(0,0),
	]
	
	codes = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
		]

	path = Path(verts, codes)

	print path.contains_point((0,0))
	
	path = Path(verts, codes)

	fig, ax = plt.subplots()
	patch = patches.PathPatch(path, facecolor='cyan', lw=2)
	ax.add_patch(patch)
	ax.set_xlim(-150, 150)
	ax.set_ylim(-150, 150)
	plt.xlabel("X distance from Centre of Mass (mm)", fontdict=None, labelpad=None)
	plt.ylabel("Y distance from Centre of Mass (mm)", fontdict=None, labelpad=None)
	plt.grid(b=True, which='major', axis='both')
	plt.text(defaultflpos[0], defaultflpos[1]-20, " FL", fontsize=18)
	plt.text(defaultfrpos[0], defaultfrpos[1]-20, " FR", fontsize=18)
	plt.text(defaultrlpos[0], defaultrlpos[1]+10, " RL", fontsize=18)
	plt.text(defaultrrpos[0], defaultrrpos[1]+10, " RR", fontsize=18)

	plt.plot(defaultchassispos[0],defaultchassispos[1], marker='o', markersize=5, color="red")

	plt.show()

def gait2():

	global flpos #mm
	global frpos #mm
	global rlpos #mm
	global rrpos #mm
	global chassispos

	#When x = 0 y = -100
	defaultflpos = [85,-114.5,0]
	defaultfrpos = [-85,-114.5,0]
	defaultrlpos = [120,53.5,0]
	defaultrrpos = [-120,53.5,0]
	defaultchassispos = [0,0,0]

	defaultrlpos[1]=defaultrlpos[1]+70
	defaultrrpos[1]=defaultrrpos[1]+70
	defaultflpos[1]=defaultflpos[1]+50
	defaultfrpos[1]=defaultfrpos[1]+50
	defaultchassispos[1]=defaultchassispos[1]

	#Working pattern, all+50, rear moves first.

	foot = "RR"

	if (foot == "RL"):

		verts = [
			(defaultflpos[0],defaultflpos[1]),
			(defaultfrpos[0],defaultfrpos[1]),
			(defaultrrpos[0],defaultrrpos[1]),
			(0,0),
		]

	if (foot == "RR"):

		verts = [
			(defaultflpos[0],defaultflpos[1]),
			(defaultfrpos[0],defaultfrpos[1]),
			(defaultrlpos[0],defaultrlpos[1]),
			(0,0),
		]

	if (foot == "FL"):

		verts = [
			(defaultfrpos[0],defaultfrpos[1]),
			(defaultrrpos[0],defaultrrpos[1]),
			(defaultrlpos[0],defaultrlpos[1]),
			(0,0),
		]

	if (foot == "FR"):

		verts = [
			(defaultflpos[0],defaultflpos[1]),
			(defaultrrpos[0],defaultrrpos[1]),
			(defaultrlpos[0],defaultrlpos[1]),
			(0,0),
		]
	


	codes = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
		]

	path = Path(verts, codes)

	print path.contains_point((0,0))
	
	path = Path(verts, codes)


	#When x = 0 y = -100
	defaultflpos2 = [85,-114.5,0]
	defaultfrpos2 = [-85,-114.5,0]
	defaultrlpos2 = [120,53.5,0]
	defaultrrpos2 = [-120,53.5,0]
	defaultchassispos2 = [0,0,0]

	verts2 = [
		(defaultflpos2[0],defaultflpos2[1]),  # left, bottom
		(defaultfrpos2[0],defaultfrpos2[1]),  # left, top
		(defaultrlpos2[0],defaultrlpos2[1]),  # right, top
		(0,0),
	]
	
	codes2 = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
		]

	path2 = Path(verts2, codes2)

	print path.contains_point((0,0))
	
	path2 = Path(verts2, codes2)


	fig, ax = plt.subplots()
	patch = patches.PathPatch(path, facecolor='cyan', lw=2)
	patch2 = patches.PathPatch(path2, facecolor='blue', lw=2)
	ax.add_patch(patch)
	ax.add_patch(patch2)
	ax.set_xlim(-150, 150)
	ax.set_ylim(-150, 200)
	plt.xlabel("X distance from Centre of Mass (mm)", fontdict=None, labelpad=None)
	plt.ylabel("Y distance from Centre of Mass (mm)", fontdict=None, labelpad=None)
	plt.grid(b=True, which='major', axis='both')
	plt.text(defaultflpos[0], defaultflpos[1]-20, " FL", fontsize=18)
	plt.text(defaultfrpos[0], defaultfrpos[1]-20, " FR", fontsize=18)
	plt.text(defaultrlpos[0], defaultrlpos[1]+10, " RL", fontsize=18)
	#plt.text(defaultrrpos[0], defaultrrpos[1]+10, " RR", fontsize=18)

	plt.plot(defaultchassispos[0],defaultchassispos[1], marker='o', markersize=5, color="red")

	plt.show()





def publisher():
	#rospy.init_node('JointPublisher', anonymous=True)
	gait2()
	#rospy.spin()


if __name__ == '__main__':
	publisher()
