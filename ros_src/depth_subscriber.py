#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('depth_sub')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ros_tkdnn.msg import yolo_coordinateArray

import numpy as np
class image_converter:

  def __init__(self):
  

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_cb)
    self.rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.rgb_cb)
    # subscribe yolo output
    self.yolo_sub = rospy.Subscriber("/yolo_output",yolo_coordinateArray,self.yolo_cb)
  
  def depth_cb(self,data):
    try:
      #(480,640,3)
      self.cv_depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
      
    except CvBridgeError as e:
      rospy.logerr(e)
    
  def yolo_cb(self,data):
    
    if len(data.results) > 0:
      # make sure the number of results
      print("=============================")
      print(len(data.results))

      self.x_center = data.results[0].x_center
      self.y_center = data.results[0].y_center
      self.w = data.results[0].w
      self.h = data.results[0].h 
      self.xmin = data.results[0].xmin 
      self.xmax = data.results[0].xmax
      self.ymin = data.results[0].ymin
      self.ymax = data.results[0].ymax 
      
      print(self.xmin, self.ymin)
      print(self.xmax, self.ymax)
    else:
      rospy.logwarn("No results")


  def rgb_cb(self,data):
    try:
      #(480,640,3)
      self.cv_rgb_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

    except CvBridgeError as e:
      rospy.logerr(e)

    self.draw()

  
  def draw(self):
    cv2.imshow("Depth window", self.cv_depth_image)
    cv2.waitKey(3)

    self.cv_rgb_image = cv2.circle(self.cv_rgb_image, (self.x_center, self.y_center),10, 255, 3)
    cv2.imshow("RGB window", self.cv_rgb_image)



    cv2.waitKey(3)


    #rospy.loginfo(data)
    # point = cv2.circle(cv_image, (50,50), 10, 65536, -1)
    # cv.imshow("rgb", rgb_image)
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
  


def main(args):
  ic = image_converter()
  rospy.init_node('depth_subscriber', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)