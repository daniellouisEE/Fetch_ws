#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import warnings
import rospy
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
#use service bc we only want to use it once at a time 
global bbcare
    globxmin= bounding_boxes.bounding_boxes[0].xmin

def callback(bounding_boxes):
    rospy.loginfo(rospy.get_caller_id() + "xmin %s,xmax %s,ymin %s, ymax %s",bounding_boxes.bounding_boxes[0].xmin,bounding_boxes.bounding_boxes[0].xmax,bounding_boxes.bounding_boxes[0].ymin,bounding_boxes.bounding_boxes[0].ymax)
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/darknet_ros/bounding_boxes',
                                    BoundingBoxes, callback)
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
