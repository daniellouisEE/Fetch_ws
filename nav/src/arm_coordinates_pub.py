#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import PlanningSceneInterface

###something really weird is happening here, it works on sub


def talker():
    count = 0.001
    pub = rospy.Publisher('arm_coordinates', Pose, queue_size=10)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_coordinates', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.6
    pose_goal.position.y = 0.3
    pose_goal.position.z = count 
    

    
    while not rospy.is_shutdown():  

        pose_goal.position.z = 1 + count
        log_str = str(pose_goal)
        rospy.loginfo(log_str)
        pub.publish(pose_goal)
        _ = raw_input()
        count += 0.1
        rate.sleep()
            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


    



