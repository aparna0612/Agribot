********************************************** IMPLEMENTATION OF THE CODE **************************************************

########## GENERAL IDEA ##########

The idea used behind the implementaion of the algorithm is that the bot will constantly move in the greenhouse on a specified path that will cover both sides of the trough of the 
tomato plants and when the bot will identify a tomato that is within a specified depth range the arm will be accutated to pick up the tomato. The bot will get to know that a tomato that is
visible in the frame is in the depth range as the TF of the tomato will only be bradcasted when the tomato comes in a specific depth range. To get the correct depth value of the tomato the
speed of the bot is reduced so that the precision of detection is increased. When we get a tomato that is within the depth range then the algorithm for the alligment of the Aruco Marker is 
triggered that will help the bot to center the aruco marker in the camera frame so that the pickup of the tomato becomes smooth, for this the TF of the aruco marker is used to allign the 
center of the marker to the center range of the camera.

##################################


######### COLOR DETECTION ALGORITHM ##########

tomatoColor = [[0, 37, 117, 0, 250, 255]]    -----> this is the HSV max and min values that are used to get perfect detection of the red color of the tomatoes

Over here when the HSV value of the color that is visible in the frame is checked with this HSV space values and if it matches a circular contour is formed around the tomato but over here the 
area thresholding is used to remove noise as follows :-

area > 600   ----> this is the area thresholding that means if the blob of red color is greater than 600 units then only the contour is formed 


In this section there is a flagDetection variable that is triggered to TRUE if the area of the blob is in the specified range 

flagDetect = True ----> this is the variable that tells the bot that the tomato is visible and now the bot need to slow down the speed to get the TF if it is in the depth range

midX = x + w // 2, midY = y + h // 2 ----> these two values are used to get the mid point of the tomato that is used for bradcasting the TF of the tomatoes

depth = depth_image[midY][midX-w//2] ----> this is very important as this the the re-adjustment of the depth value that is used for tomato detection and not for TF broadcast 
over here the depth value of one side of the tomato is taken instead of center this is done because if there is any lag in the system so it will ensure that the depth taken is of the tomato
only and not of the surrounding objects ----> because if the center is taken and due to some lag in the system the depth value is recorded a little late so by that time the tomato
could have moved out of that point but is the depth is taken of one side this will drastically reduce the chances of error

depth <= 0.80 ---> this is the depth filtering that will only bradcast the TF of the tomato that is in this range ---> this will prevent the arm from picking up the tomatoes on the other 
side of the plant and will prevent chances of damage that could be caused if the arm crashes to the plant ---> we can definitely pick up these tomatoes when we get to the other side of the plant

###############################################


############## MOTION OF THE BOT ##############

*********************************************

moveBot() ---> this function is used to control the motion of the bot 

In this function there are various stages of the motion of the bot and the stages are triggered in order that help in moving the bot in the greenhouse
Stages are quiet important in this type of motion where the bot repeatedly moves in the same location but with different attributes of motion ---> for example once the bot is at the start
location it has to move forward to enter the rows and start the process of picking up the tomatoes but at the end of the execution the bot has to return to the same start location but
now it has to print Mission Accomplished message and has to stop motion now to prevent these two stages from interfering with each other.

**********************************************

**********************************************

if flagDetect is True:      
    linear_vel = 0.02
    angular_vel = 0.0 
    Pcontroller()
else:
    linear_vel = 0.2       
    angular_vel = 0.0
    Pcontroller()   


The above code snippet is where the flagDetect is used to slow down the speed of the bot for correct detection of the depth value of the tomatoes visible.

***********************************************

***********************************************

pose[0] ---> is the x value of the bot
pose[1] ---> is the y value of the bot
pose[2] ---> is the orientation of the bot

These 3 values are used from odometry

stopMotion()    ----> this function is to stop the motion of the bot

Pcontroller() ----> this is the P-Controller that is used to help the bot maintain a constant distance from the trough so that it doesn't collide with the trough

kp = 10 ---> this is the correction factor that is used to help the bot to correct its motion quickly

************************************************

###############################################



#################### LASER ####################

'bleft':  min(min(msg.ranges[680:720]),3.2)  ----> the bleft region is defined from 680 to 720 units range to get the exact left of the bot

3.2 value is the max distance that will be given if the ranges go to infinite distance

###############################################




############# CODE IN ACTION ##################

*********************************
When the while loop is triggered it first checks that if the bot is in the perception pose or not if it is not in that pose the below if statement is triggered :-

if flagPose is True:
    flagPose = False
    ur5.arm_robot(0)
    rospy.sleep(0.1)
    flag = True

**********************************

************************************************

When the system has ensured that the bot is now in perception pose the bot starts looking for the TF of the tomatoes ---> it does this by using a for loop where it check the max number of 
contours that are formed and ----> if the contours are formed and the depth is in the range then we get the TF of the tomato and the hold value holds the object number for which the TF is 
bradcasted and then the hold value is checked as it no longer is a blank string it breaks out of the loop and we move to further acutation for picking up the tomato

for i in range(maxCounter+1):       #this loop actually checks which tomato is in the permissible depth range 
    try:
        trans = tfBuffer.lookup_transform('base_link', 'obj'+str(i), rospy.Time(0))      #for getting the TF of obj0 ----> tomato in the scene
        hold = str(i)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

    if hold != '':
        break

**************************************************

The loop can also get terminated once it completes its range and so the hold value is checked ---> if the hold value is still a blank string it means we didn't encounter any tomato in the range
so the bot keeps moving


if hold == '':  #if no tomato is visible the hold will be blank string so the bot will keep moving
    moveBot()                           #if the TF of the bot is not available then the bot has to keep moving
    velocity_msg.linear.x = linear_vel
    velocity_msg.angular.z = angular_vel
    pub.publish(velocity_msg)          #publishing the value to the bot

    '''If the tomato is not visible then the aruco id is set to an invalid id ---> this will prevent the bot from alligning to the previous marker when the tomato is visible , 
    this is very important because if it is set to some valid id as 0 then when we are trying to allign the bot to the marker it will already have TF of some valid marker and 
    it will try to move to that marker to allign itself insead of moving slowly forward and alligning to the marker that is visible or will get visible in moving forward
    instead of alligning to the previous marker'''

    ArucoId = '-1' #set to invalid id -1            -------> the aruco id is set to invalid id -1 so that it doesn't try to allign with the latest encountered aruco marker

    continue       #to continue with the next iteration if we get some exception

***************************************************


***************************************************

The code below is to get the TF of the Aruco Marker visible ----> now if the ArucoId was not set to -1 then it would be holding the TF of the Aruco Marker that was last visible and so it will
move the bot backward instead of moving forward and will allign itself to the previous plant instead of this plant ----> that is why the id was set to -1 so that it doesn't have the
TF for this id and will move forward to get the TF.

try:
    transAruco = tfBuffer.lookup_transform('sjcam_link', 'aruco'+ArucoId, rospy.Time(0))
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    velocity_msg.linear.x = 0.1             #till the time the TF of the aruco is not available then the bot is moving with a small velocity of 0.1
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    continue

***************************************************

***************************************************

The code below is for the allignment of the Aruco Marker to the center of the camera frame ---> now over here the -0.01 to 0.01 is the range in which the Marker's center should be present
but if it is not present in that range then the bot moves forward or backward accordingly to set the marker's center to the central range given ---> for this it uses the TF of the marker
to know where the center of the ID is and move the bot to allign it.


while transAruco.transform.translation.y > 0.01 or transAruco.transform.translation.y < -0.01:  #when the TF is available but the center of the Aruco is not in the range 
    if transAruco.transform.translation.y > 0.01:   #if the bot has moved forward by any chance then a negative velocity will bring it back
        velocity_msg.linear.x = -0.1
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
    elif transAruco.transform.translation.y < -0.01:        #if the bot has moved backward then a positive velocity will do the work
        velocity_msg.linear.x = 0.1
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
    transAruco = tfBuffer.lookup_transform('sjcam_link', 'aruco'+ArucoId, rospy.Time(0))    #the TF is continuously read so that we are able to allign the aruco properly

**************************************************

**************************************************

The code below is to get the TF of the tomato when the bot has completely stopped so that the TF registered is correct and without any errors


trans = tfBuffer.lookup_transform('base_link', 'obj'+hold, rospy.Time(0))

rospy.loginfo('obj'+hold+' Identified at '+ArucoId)         -----> this is to print the message 


tomato_Position_X_Transform = trans.transform.translation.x        ---> these are the TF of the tomato visible
tomato_Position_Y_Transform = trans.transform.translation.y        ---> these are the TF of the tomato visible
tomato_Position_Z_Transform = trans.transform.translation.z        ---> these are the TF of the tomato visible

**************************************************

########################################################

################### ACTUATION FOR THE TOMATO ####################

In this section basically what is being done is that first the acutation to the tomato to be picked is tried from the first perception pose but if it fails


if flagExecution is False and flagNewPose is False:       ----> this is triggered that changes the arm's pose to a new perception pose        
    ur5.arm_robot(2)       
    rospy.sleep(0.1)
    flagNewPose = True              ----> this variable tells the system that where the bot has already been taken to the new perception pose or not
    continue



if flagNewPose is True and flagExecution is False:  -----> if the bot has already been taken to the 
    continue


ur5.gripper_robot(1) ----> this is to close the gripper

ur5.gripper_robot(0) ----> this is to open the gripper


flagPose = True                ----> to set this variable to true again so that the arm positions itself correctly after picking up the tomato
flag = False                   ----> this will prevent further execution till the arm has not positioned itself correctly
hold = ''
flagNewPose = False

#################################################################