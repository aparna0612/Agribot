#!/usr/bin/env python3


import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg  # for geometry messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library
import time


linear_vel = 0.0
angular_vel = 0.0


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
    kp = 1.8
    angular_vel = kp * (regions['bleft'] - 0.60)


def forward():
    global linear_vel
    global angular_vel
    linear_vel = 0.2
    angular_vel = 0.0


def turnLeft():
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.5   #----------------------> CHANGE THIS


def stopMotion():
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0



def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
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

    start_time = time.time()
    second_turnOne = 120
    second_two = 310
    count = 0
    while not rospy.is_shutdown(): 
        current_time = time.time()
        elapsed_time = current_time - start_time
        rospy.loginfo(elapsed_time)
        if regions['bleft'] > 1.2 and regions['bright'] > 1.2 and count == 0:                #stage 1
            while regions['bleft'] > 1.2 and regions['bright'] > 1.2:
                forward()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rospy.sleep(0.1)
            
            forward()
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(1)

            count += 1
        elif (elapsed_time >= second_turnOne) and (regions['bleft'] > 1.2 and regions['bright'] > 1.2 and count == 1):
            forward()
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(3)


            turnLeft()
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(7)

            forward()
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(9)

            count += 1

        elif (elapsed_time >= second_two) and (regions['bleft'] > 1.2 and regions['bright'] > 1.2 and count == 2):
            forward()
            velocity_msg.linear.x = linear_vel + 0.1
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(3)

            stopMotion()
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(0.1)

            break

        forward()
        Pcontroller()
        velocity_msg.linear.x = linear_vel
        velocity_msg.angular.z = angular_vel
        pub.publish(velocity_msg)
        rospy.sleep(0.1)

        rate.sleep()

if __name__=='__main__':
    main()