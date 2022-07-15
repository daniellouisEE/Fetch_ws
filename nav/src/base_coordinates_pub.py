#!/usr/bin/env python
# license removed for brevity
import rospy
# import the data type "MoveBaseGoal"
from move_base_msgs.msg import MoveBaseGoal

def talker():
    pub = rospy.Publisher('chatter', MoveBaseGoal, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        out_goal = MoveBaseGoal()
        # set origin to robot 
        out_goal.target_pose.header.frame_id = "base_link" #can also  be set to "map"  
        out_goal.target_pose.pose.position.x = 1 
        out_goal.target_pose.pose.position.y = 0 
        out_goal.target_pose.pose.position.z = 0
        
        out_goal.target_pose.pose.orientation.w = 1
        
        log_str = str(out_goal.target_pose.pose.position.x) + " " + str(out_goal.target_pose.pose.position.y) + " " + str(out_goal.target_pose.pose.position.z)
        rospy.loginfo(log_str)
        pub.publish(out_goal)
        _ = raw_input()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

