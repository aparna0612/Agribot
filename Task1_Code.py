#!/usr/bin/env python3


import rospy                                            #for importing rospy
from geometry_msgs.msg import Twist                     #for getting the geometry messages of type Twist
from sensor_msgs.msg import LaserScan                   #to get the laser values
from nav_msgs.msg import Odometry                       #for getting the odometry
from tf.transformations import euler_from_quaternion    #for conversion of querternion to euler type
    

linear_vel = 0.5        #variable to hold the linear velocity of the bot
angular_vel = 0.0       #variable to hold the angular velocity of the bot

#All the flag variables define the different stages of the bot motion
flag1 = True          
flag2 = False
flag3 = False
flag4 = False
flag5 = False

pose = [0.0] * 4        #initializing the pose variable

#initialization of the regions variable
regions = {
        'bright': 0.0  ,
        'fright': 0.0  ,
        'front':  0.0  ,
        'fleft':  0.0  ,
        'bleft':  0.0  ,
    }



def forwardMotion():            #function to control the forward motion of the bot i.e. the straight linear motion of the bot
    global linear_vel
    global angular_vel
    linear_vel = 1.0
    angular_vel = 0.0


def stopMotion():               #function to stop the motion of the bot
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0



def leftTurn():                 #function to left turn the bot 
    global angular_vel
    global linear_vel
    linear_vel = 0.6
    angular_vel = 1.7



def odom_callback(data):        #callback funtion for odom
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]



def laser_callback(msg):                #callback function for laser
    global regions
    regions = {
        'bright': min(min(msg.ranges[0:20]),3.2)  ,
        'fright': min(min(msg.ranges[40:60]),3.2)  ,
        'front':  min(min(msg.ranges[80:100]),3.2)  ,
        'fleft':  min(min(msg.ranges[120:140]),3.2)  ,
        'bleft':  min(min(msg.ranges[160:180]),3.2)  ,
    }
    



def Pcontroller(stage):     #function for the proportional controller --> this has been used so that the bot is able to correct its path if it gets deflected form its path
    global angular_vel
    global regions
    kp = -1     #correction factor
    if stage == 1:          #correction for stage 1
        angular_vel = kp * (regions['bleft']-0.82)  #it reads bleft laser values and 0.82 is the expected value and the difference gives the deflection and on the bases of that it returns angular velocity
    elif stage == 3:        #correction for stage 3
        angular_vel = kp * (regions['bleft']-2.0)
    elif stage == 5:        #correction for stage 5
        angular_vel = kp* (regions['bleft']-2.0)





def control_loop():          #the control function

    rospy.init_node('ebot_controller')             #initializing node 

    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)   #setting up the publisher

    rospy.Subscriber('/ebot/laser/scan',LaserScan, laser_callback)      #setting up the subscriber

    rospy.Subscriber('/odom', Odometry, odom_callback)  #setting up the subscriber

    rate =  rospy.Rate(10)      #defining the rate for the exectuion


    velocity_msg = Twist()

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)


    #getting all the global variables
    global linear_vel
    global angular_vel
    global flag1
    global flag2
    global flag3
    global flag4
    global flag5
    global pose
    global regions



    while not rospy.is_shutdown():

        if flag1 is True:       #stage 1 condition check
            flag1 = False
            flag2 = True
            while pose[1] <= 6.0:       #pose[1] is the y value of the bot 
                forwardMotion()
                Pcontroller(1)          #calling the Proportional controller for correction
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)          #publishing the value to the bot
            

        elif flag2 is True:             #stage 2 condition check
            flag2 = False               #we turn this false so that it is not triggered again and again
            flag3 = True
            while pose[1] <= 8.0:
                forwardMotion()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
            while pose[2] >= 0:         #it is the z value of the bot 
                leftTurn()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
            while pose[2] <= -1.64:
                leftTurn()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)

        elif flag3 is True:             #stage 3 condition check
            flag3 = False
            flag4 = True
            while pose[1] >= 0:
                forwardMotion()
                Pcontroller(3)
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)

        elif flag4 is True:             #stage 4 condition check
            flag4 = False
            flag5 = True
            while pose[1] >= -0.5:
                forwardMotion()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
            while pose[2] <= -0.2:
                leftTurn()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
            while pose[0] <= 1.5:       #pose[0] contains the x value of the  bot
                forwardMotion()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)

        elif flag5 is True:             #stage 5 condition check
            flag5 = False
            while pose[2] <= 1.50:
                leftTurn()
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
            while pose[1] <= 8:
                forwardMotion()
                Pcontroller(5)
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)

            stopMotion()                #the stopMotion function has been called to stop the motion of the bot
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            break                       #we break to get out of the loop

        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


if __name__=='__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass