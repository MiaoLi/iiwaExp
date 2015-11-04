#!/usr/bin/python
# license removed for brevity

"""
Modify the python script for KUKA control to iiwa control
"""

# from openravepy import *
import rospy
import roslib
from time import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Transform, Point, Pose, Twist, Wrench
from numpy import *
from numpy import linalg as LA
from random import randint
import sys
from copy import deepcopy
# roslib.load_manifest('IIWA')
from IIWA.msg import CartState
from transformations import *

iiwaCartState = CartState()

def iiwa_cart_imp(Ind):
	global iiwaCartState
	print 'start to publish to iiwa cartesian interface'
	rospy.init_node('IIWACartImpedance', anonymous =True)

	pub1 = rospy.Publisher("/iiwa/cartesian_reference_pose", CartState, queue_size=10)
	pub2 = rospy.Publisher("/iiwa/cartesian_stiffness", Wrench,queue_size = 10)
	rospy.Subscriber("/iiwa/cartesian_state", CartState, GetiiwaCartState, queue_size = 1)

	
	rs = rospy.Rate(100)   # 100hz
	CartCmdMsg = CartState()
	StiffCmdMsg = Wrench()

	while linalg.norm(array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z]))==0:
		print("Waiting for the iiwa CartState msg")
		print("Please run the robot's topic first")
		sleep(0.1)

	CartCmdMsg=deepcopy(iiwaCartState) # get the robot's initial state
	StiffCmdMsg.force.x = 500
	StiffCmdMsg.force.y = 500
	StiffCmdMsg.force.z = 500
	StiffCmdMsg.torque.x = 200
	StiffCmdMsg.torque.y = 200
	StiffCmdMsg.torque.z = 200
    

	CircleRadius =0.03 
	CircleSegments=400
	# Set the starting point of the circle
	CircleStart= array([CartCmdMsg.pose.position.x,CartCmdMsg.pose.position.y,CartCmdMsg.pose.position.z-0.03])

	OriStart = [CartCmdMsg.pose.orientation.w,CartCmdMsg.pose.orientation.x,CartCmdMsg.pose.orientation.y,CartCmdMsg.pose.orientation.z]
	rot = quaternion_about_axis(3.14/16.0,[0,1,0])
	OriGoal = quaternion_multiply(OriStart,rot)
	# OriCurrentMat = quaternion_matrix(OriStart)				
	# tmpRot  = rotation_matrix(3.14/8, [0, 1, 0])
	# tmpGoal = dot(OriCurrentMat,tmpRot)
	# OriGoal = quaternion_from_matrix(tmpGoal)
	
	CircleCenter = array([CartCmdMsg.pose.position.x,CartCmdMsg.pose.position.y+0.03,CartCmdMsg.pose.position.z-0.03])
	print('------------start the task now -----------')

	idpert = random.randint(50, 350)
	print 'The index to perturb is ', idpert
	bApproach=True
	bPolishing=False
	bPert = False  # have perturbation or not
	bOri = False # update the orientation

	deltaApproach=0.001
	CntPolish = 0
	CntInd = 0   #counter of circles

	CircleAngle = 0
	CntPert = 0  #index during polishing
	CntApproach = 0

	tmp = CircleStart - array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z]) 

	while not rospy.is_shutdown():
		if bApproach:
			print '--------start approaching---------'
			# CartCmdMsg.pose.position.x= CircleStart[0]
			# CartCmdMsg.pose.position.y= CircleStart[1]
			# CartCmdMsg.pose.position.z= CircleStart[2]
			# CartCmdMsg.pose.position.z -= 0.0001
			CartCmdMsg.pose.position.x = iiwaCartState.pose.position.x+tmp[0]*deltaApproach/LA.norm(tmp)
			CartCmdMsg.pose.position.y = iiwaCartState.pose.position.y+tmp[1]*deltaApproach/LA.norm(tmp)
			CartCmdMsg.pose.position.z = iiwaCartState.pose.position.z+tmp[2]*deltaApproach/LA.norm(tmp)

			OriCurrent =[iiwaCartState.pose.orientation.w,iiwaCartState.pose.orientation.x,
				iiwaCartState.pose.orientation.y,iiwaCartState.pose.orientation.z]
			tmpOri=OriCurrent-OriGoal
			# update orientation
			print 'The position error is ', LA.norm(numpy.array(tmpOri))
			if LA.norm(numpy.array(tmpOri))>0.01:
				OriGoaltmp = quaternion_slerp(OriCurrent,OriGoal,0.1)
				CartCmdMsg.pose.orientation.w =OriGoaltmp[0]
				CartCmdMsg.pose.orientation.x =OriGoaltmp[1]
				CartCmdMsg.pose.orientation.y =OriGoaltmp[2]
				CartCmdMsg.pose.orientation.z =OriGoaltmp[3]

			tmp = CircleStart - array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z])
			print 'The position error is ', LA.norm(tmp)
			
			if LA.norm(tmp)<0.01 and LA.norm(numpy.array(tmpOri)) < 0.02:
				bApproach  = False
				bPolishing = True
				CircleRadius=LA.norm(CircleCenter-array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z]))            
				print 'CircleRadius is:', CircleRadius  
				pNext=zeros([3])

		if bPolishing:            
			# print '---------start polishing------------'
			pNext[0] = CircleCenter[0] + CircleRadius*sin(CircleAngle)
			pNext[1] = CircleCenter[1] - CircleRadius*cos(CircleAngle)
			pNext[2] = CircleCenter[2]
			CircleAngle = CircleAngle+2*pi/CircleSegments

			CartCmdMsg.pose.position.x = pNext[0]
			CartCmdMsg.pose.position.y = pNext[1]
			CartCmdMsg.pose.position.z = pNext[2]
			StiffCmdMsg.force.x = 500
			StiffCmdMsg.force.y = 500
			StiffCmdMsg.force.z = 200
			
			if bOri:
				"""
				#update the orientation here, from the manifold learning, the orientation is already defined as
				# the local contact frame. 
				#oriNext = ComputeDesiredOri(pNext)
				#OriCurrent =[iiwaCartState.pose.orientation.w,iiwaCartState.pose.orientation.x,
				iiwaCartState.pose.orientation.y,iiwaCartState.pose.orientation.z]
				oriNext = quaternion_slerp(OriCurrent,oriNext,0.5)
				CartCmdMsg.pose.orientation.w=oriNext[0]
				#...
				#...
				#...
				"""
				# modify the following code later to get the orientation from manifold computation				
				OriCurrent =[iiwaCartState.pose.orientation.w,iiwaCartState.pose.orientation.x,
				iiwaCartState.pose.orientation.y,iiwaCartState.pose.orientation.z]
				OriCurrentMat = quaternion_matrix(OriCurrent)				
				tmpRot  = rotation_matrix(0.02, [0, 0, 1])
				tmpGoal = dot(OriCurrentMat,tmpRot)
				OriGoal = quaternion_from_matrix(tmpGoal)				
				if CntInd == 2:
					OriGoal = quaternion_slerp(OriCurrent,OriGoal,0.5)
					CartCmdMsg.pose.orientation.w=OriGoal[0]
					CartCmdMsg.pose.orientation.x=OriGoal[1]
					CartCmdMsg.pose.orientation.y=OriGoal[2]
					CartCmdMsg.pose.orientation.z=OriGoal[3]	



			if CntInd < Ind and CircleAngle > 2*pi:
				# repeat for Ind times
				if CntInd == 2:
					CircleCenter[0] = CircleCenter[0]+0.015
				elif CntInd == 3:
					CircleCenter[0] = CircleCenter[0] - 0.05
					CircleCenter[1] = CircleCenter[1]+0.005 # change the circle center each trial

				CircleRadius=LA.norm(CircleCenter-array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z]))            
				print 'CircleRadius is:', CircleRadius
				pNext=zeros([3])
				CircleAngle=0
				CntInd=CntInd+1  #Finish one circle

			if CntInd==Ind:
				bPolishing= False  # stop polishing
				break

			CntPert = CntPert+1
			if bPert:
				if idpert <= CntPert <= idpert+10:
					pNext[0] = iiwaCartState.pose.position.x
					pNext[1] = iiwaCartState.pose.position.y
					pNext[2] = iiwaCartState.pose.position.z + 0.02
					CartCmdMsg.pose.position.x = pNext[0]
					CartCmdMsg.pose.position.y = pNext[1]
					CartCmdMsg.pose.position.z = pNext[2] 

				if idpert+11 <= CntPert:
					# compute the projected position if necessary
					# CircleStart=array([iiwaCartState.pose.position.x,iiwaCartState.pose.position.y,iiwaCartState.pose.position.z-0.03])
					bApproach = True
					bPolishing= False

		pub1.publish(CartCmdMsg)
		pub2.publish(StiffCmdMsg)
		rs.sleep()

def GetiiwaCartState(msg):
	global iiwaCartState
	iiwaCartState = msg

def ComputeDesiredOri(pos):
	# Given the position, compute the orientation of local frame
	# convert it to quternion
	pass
	print 'compute the orientation using learned manifold'


if __name__ == '__main__':
	try:
		if len(sys.argv)>1:
			Ind = int(sys.argv[1])
		else:
			Ind = 100  # any big number
		raw_input('press to continue')  
		iiwa_cart_imp(Ind)
	except rospy.ROSInterruptException: pass