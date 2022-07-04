#!/usr/bin/env python
import rospy								#importing rospy
from geometry_msgs.msg import Twist			#importing geometry msgs as it has Twist messages for the cmd_vel topic
from turtlesim.msg import Pose				#for getting the pose of the turtle i.e. the x,y and theta value of the turtle
import sys

line_vel = 2.0				#linear velocity if set to 2
angu_vel = -2.0			#angular velocity if set to -2 initially

positionY = 100.0	#default values to keep it out of range of the set triggers
positionX = 100.0

flag1 = False		#this is to get the state if the turtle is following the lower circle
flag2 = False	#this is to get the state if the turtle is following the upper circle


def pose_callback(pose):	#this function is to get the position of the turtle
	global positionY
	global positionX
	positionY = pose.y
	positionX = pose.x


	


def move_turtle():
	rospy.init_node('move_turtle', anonymous = False)		#for the initialization of the node
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)	#for setting the publisher
	rospy.Subscriber('/turtle1/pose',Pose,pose_callback)		#for setting the subscriber
	rate = rospy.Rate(60)		#the rate of the function is set to 60 Hz
	
	vel = Twist()		#for holding the Twist messages
	while not rospy.is_shutdown():	#for running the loop till it is not shut down

		global positionY	#for getting the state of all the variables in this method
		global positionX
		global line_vel
		global angu_vel
		global flag1
		global flag2

		if positionY <= 4:	#this conditions that the turtle is in the lower circle trajectory
			flag1 = True


		if flag1 is True and positionY >= 5.5:#this condition condition is to change the velocity only after the trutle has completed the lower circle
			if positionX >= 5.5:
				flag1 = False
				line_vel = 2.0
				angu_vel = 2.0

		if angu_vel == 2.0 and positionY >= 7:#to confirm that the turtle is in the upper circle
			flag2 = True

		if flag2 is True and positionY <= 5.55:#to stop the turtle after it has completed the upper circle
			vel.linear.x = 0
			vel.angular.z = 0
			pub.publish(vel)
			rospy.loginfo("Goal Reached")	#to log the goal reached information 
			break

		rospy.loginfo("Moving in a circle")
		vel.linear.x = line_vel	#for initializing the correct velocity values to the turtle
		vel.linear.y = 0
		vel.linear.z = 0
		
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = angu_vel
		
		pub.publish(vel)		#for publishing the velocity information 
	rate.sleep()	#to keep the node alive till the shutdown is not triggered

	
		
if __name__=='__main__':	#main method
	try:
		move_turtle()			#calling the function
	except rospy.ROSInterruptException:
		pass