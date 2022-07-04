#!/usr/bin/env python3


import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg  # for geometry messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library

linear_vel = 0.0
angular_vel = 0.0
stage = [0] * 9

orient = 10.0


regions = {
    'bleft' : 0.0,
    'bright' : 0.0,
}

def laser_callback(msg):
    global regions
    regions = {
        'bleft' : min(min(msg.ranges[680:720]),1.5),
        'bright' : min(min(msg.ranges[0:40]),1.5),
    }


def Pcontroller():
    global angular_vel
    kp = 2.0
    angular_vel = kp * (regions['bleft'] - 0.55)


def forward():
    global linear_vel
    global angular_vel
    linear_vel = 0.2
    angular_vel = 0.0

def turnLeft():
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.05

def stopMotion():
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0

def orientRight():
    global  orient
    global  angular_vel
    if float(orient) > 0.1:
        angular_vel = -0.05
    elif float(orient) < -0.1:
        angular_vel = 0.05



def odom_callback(data):
    global orient
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]
    # rospy.loginfo(pose[2])
    orient = pose[2]


def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)  # setting up the subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)
    global linear_vel
    global angular_vel
    global orient
    global stage
    global regions

    rate = rospy.Rate(5)
    flagStage0 = False
    flagStage3 = False
    flagLastStage = False
    while not rospy.is_shutdown():            
        if (1 < orient < 2) and (flagStage0 is False):
            flagStage0 = True
            while regions['bleft'] > 1.0:
                rospy.loginfo(1)
                forward()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rospy.sleep(0.1)

            forward()
            rospy.loginfo(6)
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(2)
        elif (-1 < orient < 1) and (flagStage3 is False):
            flagStage3 = True
            rospy.loginfo(2)
            if orient > 0.2:
                while orient > 0.2:
                    rospy.loginfo(8)
                    linear_vel = 0.1
                    angular_vel = -0.3
                    velocity_msg.linear.x = linear_vel
                    velocity_msg.angular.z = angular_vel
                    pub.publish(velocity_msg)
                    rospy.sleep(0.1)
            elif orient < -0.2:
                while orient < -0.2:
                    rospy.loginfo(9)
                    linear_vel = 0.1
                    angular_vel = 0.3
                    velocity_msg.linear.x = linear_vel
                    velocity_msg.angular.z = angular_vel
                    pub.publish(velocity_msg)
                    rospy.sleep(0.1)

            forward()
            rospy.loginfo(3)
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(4)
        elif (regions['bright'] > 1) and (regions['bleft'] > 1) and (flagStage3 is True) and (orient >= -1.6 and orient <= -1.5):
            flagLastStage = True
            forward()
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(3)

            stopMotion()
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(0.1)
            break
            

        if flagLastStage is False :
            forward()
            Pcontroller()
            rospy.loginfo(5)
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(0.1)

        rate.sleep()
if __name__=='__main__':
    main()