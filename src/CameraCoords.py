#!/usr/bin/python
# -*- coding: utf-8 -*-

import warnings
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32

from pprint import pprint
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2 as sensor_msgs_pc2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
import pyrealsense2 as rs2
import cv2
import numpy as np



class BoundingBox:

    Prob = 0
    Xmin = 0
    Ymin = 0
    Xmax = 0
    Ymax = 0
    Id = 0
    Class = ''
    XMidPoint = 0
    YMidPoint = 0


class BoundingBoxDepthData:

    def __init__(self, topic):
        self.XMidPoint = 0
        self.YMidPoint = 0
        self.topic = topic
        self.bridge = CvBridge()
        self.Class = 0
        self.DepthData = 0

        self.wantedstring = 'all'

        self.RawDataString = 0
  
        self.CenterIdx = 0

        self.BoundingBox = BoundingBox()

        self.sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                    BoundingBoxes, self.initrawstring)

        self.sub_info = rospy.Subscriber('/head_camera/depth/image', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None

    def initrawstring(self, BoundingBoxes):

       self.RawDataString = BoundingBoxes.bounding_boxes
       self.RawDataString = str(self.RawDataString)   
       self.RawDataString = self.RawDataString.split()
       self.callback1(BoundingBoxes)


    def callback1(self, BoundingBoxes):
        
        self.BoundingBox.Prob = float(self.RawDataString[1])
        self.BoundingBox.Xmin = int(self.RawDataString[3])
        self.BoundingBox.Ymin = int(self.RawDataString[5])
        self.BoundingBox.Xmax = int(self.RawDataString[7])
        self.BoundingBox.Ymax = int(self.RawDataString[9])
        self.BoundingBox.Id = int(self.RawDataString[11])
        self.BoundingBox.Class = self.RawDataString[13]
        self.BoundingBox.XY = np.array([ \
( (int(self.RawDataString[3]) + int(self.RawDataString[7]) ) / 2), \
( (self.BoundingBox.Xmin + self.BoundingBox.Xmin) / 2) ])
        try:
          self.BoundingBox.Class = self.BoundingBox.Class.replace(']', '')
          self.BoundingBox.Class = self.BoundingBox.Class.replace('"', '')
        except:
          temp = 1

        self.BoundingBox.XMidPoint = (float(self.BoundingBox.Xmax) + float(self.BoundingBox.Xmin))/2
        self.BoundingBox.YMidPoint = (float(self.BoundingBox.Ymax) + float(self.BoundingBox.Ymin))/2

        self.BoundingBox.XMidPoint = int(self.BoundingBox.XMidPoint)
        self.BoundingBox.YMidPoint = int(self.BoundingBox.YMidPoint)

        rospy.Subscriber('/head_camera/depth/image_rect_raw',\
                         msg_Image, self.callback3)
        ##'/camera/depth/image_rect_raw',\
        # 
        self.BoundingBox.target = np.array([[x, y] for x in range(self.BoundingBox.Xmin, self.BoundingBox.Xmax, 2) for y in range(self.BoundingBox.Ymin, self.BoundingBox.Ymax, 2)])

    def callback3(self, Data):
        #make YUGE array to get the median depth value in bouding box
        pix = (self.BoundingBox.XMidPoint, self.BoundingBox.YMidPoint)

        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(Data)
        DepthArray = np.array(depth_image, dtype=np.float32)

        bigdepth = np.array(DepthArray[\
self.BoundingBox.target[:,1], self.BoundingBox.target[:,0] ]) 
        self.DepthData = np.percentile(bigdepth[bigdepth !=0],3)

        if self.intrinsics:
          depth2 = depth_image[pix[1], pix[0]]
          result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], self.DepthData)
          sys.stdout.write(" Topic: %s: \n Coordinates: (%f, %f, %f) (mm) \n \r" % (self.topic, result[0], result[1], result[2]))

               
        sys.stdout.write("BoundingBox(%d, %d) Class: %s Depth: %f(mm) \r" \
               % (pix[0],
                  pix[1],
                  self.BoundingBox.Class, self.DepthData))
        sys.stdout.write("\033[1A")
        sys.stdout.write("\033[1A")
        sys.stdout.write("\x1b[1K")
        sys.stdout.flush()

        pub = rospy.Publisher('XData', Float32, queue_size=10)
        pub.publish(result[0]/1000)
        pub = rospy.Publisher('YData', Float32, queue_size=10)
        pub.publish(result[1]/1000)
        pub = rospy.Publisher('ZData', Float32, queue_size=10)
        pub.publish(result[2]/1000)
        pub = rospy.Publisher('Depth_Data', Int16, queue_size=10)
        pub.publish(self.DepthData)
        pub = rospy.Publisher('Class', String, queue_size=10)
        pub.publish(self.BoundingBox.Class)

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    rospy.loginfo('Depth Data Begin: ')
    rospy.init_node('BoundingBoxData', anonymous=True)

    topic = '/darknet_ros/bounding_boxes'

    listener = BoundingBoxDepthData(topic)

    rospy.spin()
