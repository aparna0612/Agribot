#! /usr/bin/env python

import rospy            #to import the rospy for python work
import sys
import copy
import moveit_commander     #for moveit
import moveit_msgs.msg
import geometry_msgs.msg            #for the import of the geometry messages
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('tomato_picker', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)             #for making the acutation group to arm so that this script can control the arm
        self._hand_group = moveit_commander.MoveGroupCommander("gripper")  #this is to initialize a new group called the _hand_group so that the opening and closing of the gripper can be controlled
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)        #this is to initialize a publisher

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()             #for getting the planning frame
        self._eef_link = self._group.get_end_effector_link()        #for getting the end effector link
        self._group_names = self._robot.get_group_names()           #for getting the group name of the robot


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')         #this is used to log information to the terminal
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def gripper_robot(self,state):          #this function is used to control the gripper and it accepts the state variable for setting the state of the gripper
        if state == 1:                      #state 1 means that the gripper will be in the closed state
            self._hand_group.set_named_target("close")      #this is to set the target to a pre-defined pose of the gripper as close
            plan2 = self._hand_group.go()               #this line plans and executes the instructions given by moveit
        elif state == 0:                #state 0 means that the gripper will be in the open state
            self._hand_group.set_named_target("open")           #this sets the state to open
            plan2 = self._hand_group.go()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()              #this is to get the current joint states
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)           #this is to set the joint values to a particular state
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):                         #this is used to display whether a plan exists or not
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):                          #this is used for destruction
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    #for the first tomato ---> middle one

    lst_joint_angles_1 = [math.radians(90),                 #shoulder_pan_joint ---> value
                          math.radians(30),                 #shoulder_lift_joint ---> value
                          math.radians(-30),                #elbow_joint ---> value
                          math.radians(0),                  #wrist_1_joint ---> value
                          math.radians(10),                 #wrist_2_joint ---> value
                          math.radians(0)]                  #wrist_3_joint ---> value

    lst_joint_angles_2 = [math.radians(90),
                          math.radians(42),
                          math.radians(-42),
                          math.radians(0),
                          math.radians(10),
                          math.radians(0)]


    lst_joint_angles_3 = [math.radians(0),
                          math.radians(20),
                          math.radians(-20),
                          math.radians(0),
                          math.radians(10),
                          math.radians(0)]


    #for the second tomato ----> top one

    lst_joint_angles_4 = [math.radians(110),
                          math.radians(30),
                          math.radians(-40),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]


    lst_joint_angles_5 = [math.radians(115),
                          math.radians(35),
                          math.radians(-55),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]


    lst_joint_angles_6 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]


    # for the third tomato ----> bottom one

    lst_joint_angles_7 = [math.radians(100),
                          math.radians(50),
                          math.radians(-10),
                          math.radians(-10),
                          math.radians(0),
                          math.radians(0)]



    lst_joint_angles_8 = [math.radians(100),
                          math.radians(50),
                          math.radians(-10),
                          math.radians(-30),
                          math.radians(0),
                          math.radians(0)]



    lst_joint_angles_9 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]




    while not rospy.is_shutdown():

        # #Actuation for the first tomato

        ur5.set_joint_angles(lst_joint_angles_1)            #for the execution for the first set of actuation
        rospy.sleep(0.1)                                    #to add delay 

        ur5.set_joint_angles(lst_joint_angles_2)
        rospy.sleep(0.1)
        ur5.gripper_robot(1)                                #this is to close the gripper
        rospy.sleep(0.1)

        ur5.set_joint_angles(lst_joint_angles_3)
        rospy.sleep(0.1)
        ur5.gripper_robot(0)                                #this is to open the gripper
        rospy.sleep(0.1)


        #Actuation for the second tomato

        ur5.set_joint_angles(lst_joint_angles_4)
        rospy.sleep(0.1)

        ur5.set_joint_angles(lst_joint_angles_5)
        rospy.sleep(0.1)
        ur5.gripper_robot(1)
        rospy.sleep(0.1)


        ur5.set_joint_angles(lst_joint_angles_6)
        rospy.sleep(0.1)
        ur5.gripper_robot(0)
        rospy.sleep(0.1)



        #Actuation for the third tomato

        ur5.set_joint_angles(lst_joint_angles_7)
        rospy.sleep(0.1)

        ur5.set_joint_angles(lst_joint_angles_8)
        rospy.sleep(0.1)
        ur5.gripper_robot(1)
        rospy.sleep(0.1)

        ur5.set_joint_angles(lst_joint_angles_9)
        rospy.sleep(0.1)
        ur5.gripper_robot(0)
        rospy.sleep(0.1)

        break                                       #break to get out of the while loop

    del ur5                                         #for deleting the object of the class formed


if __name__ == '__main__':                          #this is the main method
    main()