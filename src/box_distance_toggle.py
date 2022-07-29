#!/usr/bin/python

from __future__ import print_function

import sys
import rospy
from std_srvs.srv import Trigger, TriggerRequest

def box_toggle_client():
    rospy.init_node('box_distances_client')
    rospy.wait_for_service('/compute_distance')
    box_distance_service = rospy.ServiceProxy('/compute_distance', Trigger)
    toggle = TriggerRequest()
    result = box_distance_service(toggle)
    print(result)

if __name__ == '__main__':
  try:
      box_toggle_client()
  except rospy.ROSInterruptException:
      pass

