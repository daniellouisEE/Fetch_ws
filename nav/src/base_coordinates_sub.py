#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib

def callback(target_goal):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(target_goal.target_pose.pose.position.x) + " " + str(target_goal.target_pose.pose.position.y) + " " + str(target_goal.target_pose.pose.position.z))
    ac.send_goal(target_goal)
    ac.wait_for_result()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    global ac 
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while (not ac.wait_for_server(rospy.Duration(5.0))):
        ROS_INFO("Waiting for the move_base action server to come up")
        
    rospy.Subscriber("chatter", MoveBaseGoal, callback)
  
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     listener()
   
