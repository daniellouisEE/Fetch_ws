#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import time 
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes, AttachedCollisionObject
from moveit_python import PlanningSceneInterface

def callback(arm_target_pose):
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(arm_target_pose)) 
        
  
  
    finally:
        
        move_group.clear_pose_targets()
        
        
def listener():

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
    rospy.init_node('listener', anonymous=True)
    #robot = moveit_commander.RobotCommander()
    group_name = "arm_with_torso"
    global move_group
    move_group = moveit_commander.MoveGroupCommander(group_name)
        

    rospy.Subscriber("chatter", Pose, callback)
  
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()            
if __name__ == '__main__':
    #try:
        listener()
    #finally:
