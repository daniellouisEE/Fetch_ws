#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import warnings
import rospy
import numpy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import cv2 as cv2

def callback(head_camera):
    data_as_array =  numpy.frombuffer(head_camera.data, numpy.uint16)
    data_as_array.shape = (480, 640, 1)
    #avg_dist = data_as_array[y_min:y_max, x_min:x_max].avg()
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data_as_array.nonzero())
    #cv2.imshow('image recived', data_as_array.shape)
    #cv2.waitKey()
    
    
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/head_camera/depth_registered/image', msg_Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
