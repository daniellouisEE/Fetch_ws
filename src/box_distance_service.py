#! /usr/bin/env python
import rospy                                      # the main module for ROS-python programs
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
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
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import itertools as it
                                                  # ...Other types are available, and you can create
#global bbcare
   # globxmin= bounding_boxes.bounding_boxes[0].xmin
def depth_image_to_pointcloud(depth_image):
    u = numpy.tile(numpy.arange(depth_image.shape[1]), (depth_image.shape[0], 1))
    v = numpy.tile(numpy.arange(depth_image.shape[0]), (depth_image.shape[1], 1)).T

    x_over_z = (c_x - u) / focal_x
    y_over_z = (c_y - v) / focal_y

    z = depth_image / numpy.sqrt(1 + numpy.square(x_over_z) + numpy.square(y_over_z))
    x = x_over_z * z
    y = y_over_z * z

    point_cloud = numpy.zeros((depth_image.shape[0], depth_image.shape[1], 3))
    point_cloud[:, :, 0] = x
    point_cloud[:, :, 1] = y 
    point_cloud[:, :, 2] = z

    return point_cloud
    
depth_as_array = None
Xmin = None
Xmax = None
Ymin = None
Ymax = None

focal_x = 537.7739387075931
focal_y = focal_x

c_x = 317.7413040471659
c_y = 224.2998926313842


def rgb_box_data(bounding_boxes):
    global Xmin, Xmax, Ymin, Ymax
    #rospy.loginfo(rospy.get_caller_id() + "xmin %s,xmax %s,ymin %s, ymax %s",bounding_boxes.bounding_boxes[0].xmin,bounding_boxes.bounding_boxes[0].xmax,bounding_boxes.bounding_boxes[0].ymin,bounding_boxes.bounding_boxes[0].ymax)
    Xmin = bounding_boxes.bounding_boxes[0].xmin
    Xmax = bounding_boxes.bounding_boxes[0].xmax
    Ymin = bounding_boxes.bounding_boxes[0].ymin
    Ymax = bounding_boxes.bounding_boxes[0].ymax
def depth_data(head_camera):
    global depth_as_array 
    depth_as_array =  numpy.frombuffer(head_camera.data, numpy.uint16)
    depth_as_array.shape = ((480, 640))
#   data_as_array.nonzero()
    #avg_dist = data_as_array[y_min:y_max, x_min:x_max].avg()
   # cv2.imshow('image recived', depth_as_array)
   # cv2.waitKey()                                                      # custom types
def trigger_response(request):
    
    #Callback function used by the service server to process
    #requests from clients. It returns a TriggerResponse
    # for i in range(Ymin,Ymax):
    #depth = depth_as_array[Ymin:Ymax, Xmin:Xmax].mean()
    distances = depth_image_to_pointcloud(depth_as_array)
    distances = distances[Ymin:Ymax, Xmin:Xmax].mean(axis=0).mean(axis=0)
    print(distances, distances.shape)
    #   #print(depth_x[0])
    #   for j in range(Xmin,Xmax):  
    #       depth_x = depth_as_array[j]
    #      print(depth_x[0])
    

                
    #depth = depth_as_array
    #print(depth_as_array[Xmin])
    return TriggerResponse(
        success=True,
        message="service is up and running!"
    )
#service trigger
rospy.init_node('box_distances')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/compute_distance', Trigger, trigger_response         # type, and callback
)
#depth data
rospy.Subscriber('/head_camera/depth_registered/image', msg_Image, depth_data)
#global depth_array

#RGB box data 
rospy.Subscriber('/darknet_ros/bounding_boxes',
                                    BoundingBoxes, rgb_box_data)
   
rospy.spin()                                       
#if __name__ == '__main__':
  #  trigger_response(request)
