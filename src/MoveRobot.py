#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo

import warnings
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image as msg_Image
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from pprint import pprint
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import rospy
import time
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

import image_geometry, numpy
import tf
from tf.transformations import euler_from_quaternion
import copy


class Tf_cameralinkmap:

    def __init__(self, topic):
        self.PXCoord = 0
        self.PYCoord = 0
        self.DepthData = 0 
        self.Class = 0
        self.topic = topic
        self.bridge = CvBridge()
   
        self.CurrentX_Position = 0
        self.CurrentY_Position = 0
        self.CurrentZ_Position = 0

        self.CurrentX_Orientation = 0 
        self.CurrentY_Orientation = 0
        self.CurrentZ_Orientation = 0
        self.CurrentW_Orientation = 0

        self.SavedX_Position = 0
        self.SavedY_Position = 0
        self.SavedZ_Position = 0

        self.SavedX_Orientation = 0 
        self.SavedY_Orientation = 0
        self.SavedZ_Orientation = 0
        self.SavedW_Orientation = 0

        self.YCoordTest = 0

        self.XCoord = 0
        self.YCoord = 0

        self.Odom = 0
 
        self.PXCoord = 0
        self.PYCoord = 0
        self.PZCoord = 0
        

        self.Sub_DepthData = rospy.Subscriber("Depth_Data", Int16, self.GetDepthData)
        self.Sub_PXCoord = rospy.Subscriber("XData", Float32, self.GetPXCoord)
        self.Sub_PYCoord = rospy.Subscriber("YData", Float32, self.GetPYCoord)
        self.Sub_PZCoord = rospy.Subscriber("ZData", Float32, self.GetPZCoord)
        self.Sub_Class = rospy.Subscriber("Class", String, self.GetClass)
        self.Sub_Odom = rospy.Subscriber("/odom", Odometry, self.GetOdom)

        self.WaitingCounter = 0
        self.secondwaitingcounter = 0
        
   

    def GetDepthData(self, Depth_Data):
      self.DepthData = Depth_Data.data  


    def GetPXCoord(self, XData):
      self.PXCoord = XData.data


    def GetPYCoord(self, YData):
      self.PYCoord = YData.data


    def GetPZCoord(self, ZData):
      self.PZCoord = ZData.data

  
    def GetClass(self, Class):
      self.Class = Class.data
      if self.secondwaitingcounter < 3:
        self.secondwaitingcounter = self.secondwaitingcounter + 1
      else:
        self.UserChoice()

    def GetOdom(self, Odom):
      self.Odom = Odom
      

    def UserChoice(self):
     #clean up print statement 
     try:
       self.Class = self.BoundingBox.replace(']', '')
       self.Class = self.BoundingBox.replace('"', '')
     except:
       tempy = 1

     #print object being detect as well as its depth
     print("Object: [%s] | Depth %f m " % ( self.Class, float(self.DepthData)/1000))

     #user enters wether or not they want to continue with found depth/object
     userchoice = raw_input("Continue With This Depth? (Y/N) ")
  

     if (userchoice == 'y'):
       self.Tf2()
     else:
       return

    #function to get map coorindates from the depth camera coordinates
    def Tf2(self):

       #init transform
       tfBuffer = tf2_ros.Buffer()
       listener = tf2_ros.TransformListener(tfBuffer)
      
       #find the x/y coordinates on the base_link (should be map but this was giving errors)
       try:
         trans = tfBuffer.lookup_transform('camera_depth_optical_frame', 'base_link', rospy.Time(), rospy.Duration(1))

         
         (rot, pitch, yaw) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]) 

         #do transform math to find coordinates
         self.XCoord = trans.transform.translation.x + (self.PZCoord*math.cos(yaw)) - self.PXCoord*math.sin(yaw) 
         self.YCoord = trans.transform.translation.y - (self.PXCoord*math.cos(yaw)) - self.PZCoord*math.sin(yaw) 

         # print found coordinates
         print('(X,Y) | (%f, %f) \r' % (self.XCoord, self.YCoord ))
     
         #self.PublishRvizPoint()    
         user_move = raw_input("Move to Coordinates? (Y/N) ")

         if user_move == 'y':
            self.MoveToPosition()
         else:
            self.UserChoice()
       
       #error where sometimes the transform (trans) cannot be found
       #unkown what causes it, but if it happens a bunch in a row this will stop it from appearing in the console
       #to fix just change a variable (move the object, reset the camera, reset gmapping, or reset the odom)
         self.WaitingCounter = 0
       except ( tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         print("Minor Error")
         if(self.WaitingCounter == 10):
           self.WaitingCounter = 0
           print("Minor Error | To FIX -> MoveObject/ResetCamera/ResetGmapping/ResetOdom | ")
           
         self.WaitingCounter = self.WaitingCounter + 1
         self.Tf2()

    #function to get current position of the robot
    def GetCurrentPosition(self):
     
        #gets the current position of the robot
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        
        self.CurrentX_Position = self.Odom.pose.pose.position.x
        self.CurrentY_Position = self.Odom.pose.pose.position.y
        self.CurrentZ_Position = self.Odom.pose.pose.position.z
  
        self.CurrentX_Orientation = self.Odom.pose.pose.orientation.x
        self.CurrentY_Orientation = self.Odom.pose.pose.orientation.y
        self.CurrentZ_Orientation = self.Odom.pose.pose.orientation.z
        self.CurrentW_Orientation = self.Odom.pose.pose.orientation.w

        #saves the current positon and orientation (used to move robot back to original position)

        self.SavedX_Position = self.Odom.pose.pose.position.x
        self.SavedY_Position = self.Odom.pose.pose.position.y
        self.SavedZ_Position = self.Odom.pose.pose.position.z

        self.SavedX_Orientation = self.Odom.pose.pose.orientation.x
        self.SavedY_Orientation = self.Odom.pose.pose.orientation.y
        self.SavedZ_Orientation = self.Odom.pose.pose.orientation.z
        self.SavedW_Orientation = self.Odom.pose.pose.orientation.w

    #function to move the robot to given coordinates
    def MoveToPosition(self):

        #init client/goal pose 
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        initial_pose = PoseWithCovarianceStamped()
     
        #get current robot position
        self.GetCurrentPosition()
        
        #set new pose goal (coordinates found in Tf function)
        goal.target_pose.pose.position.x = self.XCoord
        goal.target_pose.pose.position.y = self.YCoord
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        #send the goal to the client
        client.send_goal(goal)
        client.wait_for_result()
        print('Robot Moved To Goal\n')
        moveback = raw_input("Move Robot Back? (Y/N)\n")
 
        #move the robot back to its original position ( saved position from the GetCurrentPosition() function)
        if moveback == 'y':
          print('Moving Robot to (%f, %f) ' % (self.SavedX_Position, self.SavedY_Position))
          goal.target_pose.header.frame_id = "map"
          goal.target_pose.pose.position.x = self.SavedX_Position
          goal.target_pose.pose.position.y = self.SavedY_Position
          goal.target_pose.pose.position.z = 0
          goal.target_pose.pose.orientation.x = 0
          goal.target_pose.pose.orientation.y = 0
          goal.target_pose.pose.orientation.z = self.SavedZ_Orientation
          goal.target_pose.pose.orientation.w = self.SavedW_Orientation 
          client.send_goal(goal)
          client.wait_for_result()
          print('Robot Moved To Goal\n')
     
     
    #not working rn 
    def PublishRvizPoint(self):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        initial_pose = PoseWithCovarianceStamped()

        # Get current position
        self.CurrentX_Position = self.Odom.pose.pose.position.x
        self.CurrentY_Position = self.Odom.pose.pose.position.y
        self.CurrentZ_Position = self.Odom.pose.pose.position.z
  
        # Get current orientation
        self.CurrentX_Orientation = self.Odom.pose.pose.orientation.x
        self.CurrentY_Orientation = self.Odom.pose.pose.orientation.y
        self.CurrentZ_Orientation = self.Odom.pose.pose.orientation.z
        self.CurrentW_Orientation = self.Odom.pose.pose.orientation.w
     
        publish_marker = rospy.Publisher('/visualization_marker', Marker, queue_size = 2)
        marker = Marker() 
        marker.header.frame_id = "/base_link"
        marker.ns = self.Class        
        marker.id = 0

        marker.action = marker.ADD
        
        

        # marker scale
        marker.scale.x = 3
        marker.scale.y = 3
        marker.scale.z = 3

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0.7071068
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0.7071068

        # marker position
        marker.pose.position.x = self.XCoord
        marker.pose.position.y = self.YCoord
        marker.pose.position.z = 1
        # Publish
        publish_marker.publish(marker)

        # Add Label
        test = Marker()
        
        test.header.frame_id = "/map"
        test.action = test.ADD
        test.type = test.TEXT_VIEW_FACING
        marker.ns = self.Class        
        marker.id = 1
        test.text = self.Class

        test.pose.orientation.x = self.Odom.pose.pose.orientation.x
        test.pose.orientation.y = self.Odom.pose.pose.orientation.y
        test.pose.orientation.z = self.Odom.pose.pose.orientation.z
        test.pose.orientation.w = self.Odom.pose.pose.orientation.w

        # Label scale
        test.scale.x = 1.0  
        test.scale.y = 1.0
        test.scale.z = 1.0

        # Label color
        test.color.a = 1.0
        test.color.r = 1.0
        test.color.g = 1.0
        test.color.b = 0.0

        # Label position
        test.pose.position.x = self.XCoord
        test.pose.position.y = self.YCoord
        test.pose.position.z = self.CurrentZ_Position + 2
        publish_marker.publish(test)




   
if __name__ == '__main__':
    rospy.init_node('Tf_cameralinkmap', anonymous=True)


    topic = '/Depth_Data'
    listener = Tf_cameralinkmap(topic)
    rate = rospy.Rate(10.0)
    rospy.spin()




