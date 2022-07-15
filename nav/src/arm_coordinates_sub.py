#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import time 
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import PlanningSceneInterface

def callback(arm_target_pose):
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(arm_target_pose)) 
                                                        
          
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)     
        #needs to be generated each time in subscriber or else it will not follow the trajectory of the torso                                                                                                                               
        base = PlanningSceneInterface("base_link")
        torso = PlanningSceneInterface("torso_lift_link")
        head = PlanningSceneInterface("torso_lift_link")
        face = PlanningSceneInterface("head_tilt_link")
        
        #ground protection
        base.addCube("my_front_ground", 2, 1.1, 0.0, -0.97)
        base.addCube("my_back_ground", 2, -1.2, 0.0, -0.97)
        base.addCube("my_left_ground", 2, 0.0, 1.2, -0.97)
        base.addCube("my_right_ground", 2, 0.0, -1.2, -0.97)
        face.addBox("face", 0.0, 0.3, 0.19, 0.11, 0.0, 0.0)
        #link face to torso instead of headtilt because we need the box to go up and down with the torso
        #robot body protection  
        base.addBox("base", 0.3, 0.55, 0.05, 0.15, 0.0, 0.36)
        base.addBox("base_right",0.3, 0.05, 0.35, 0.15, -0.25, 0.23)
        base.addBox("base_left", 0.3, 0.05, 0.35, 0.15, 0.25, 0.23)
        base.addBox("base_front", 0.05, 0.46, 0.35, 0.35, 0.0, 0.23)
        head.addBox("head", 0.4, 0.4, 0.0, 0.1, 0.0, 0.8)
        torso.addBox("torso_left", 0.2, 0.0, 1.0, 0.0, 0.25, 0.5)
        torso.addBox("torso_right", 0.2, 0.0, 1.0, 0.0, -0.25, 0.5)
            
        move_group.set_pose_target(arm_target_pose)
        plan = move_group.go()
            
        print(plan)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        move_group.execute(plan)
        
    finally:
        
        move_group.clear_pose_targets()
        
def listener():

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
    rospy.init_node('arm_coordinates', anonymous=True)
    #robot = moveit_commander.RobotCommander()
    group_name = "arm_with_torso"
    global move_group
    move_group = moveit_commander.MoveGroupCommander(group_name)
        

    rospy.Subscriber("arm_coordinates", Pose, callback)
  
    
    
    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    finally:
        os.system("roslaunch nav tuck_arm.launch")
        return     
if __name__ == '__main__':
    listener()
    
        
                                     
