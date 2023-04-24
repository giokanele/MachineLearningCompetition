#!/usr/bin/env python3

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


 # Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0
holder =0




class image_converter:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/R1/cmd_vel",Twist, queue_size=1)
    self.license_pub = rospy.Publisher("/license_plate",String, queue_size=1)

    self.twist = Twist()


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.speed_sub = rospy.Subscriber("/R1/cmd_vel", Twist, self.speed_callback)

  def speed_callback(self, data):
    self.twist = data
    print("Linear speed: "+str(self.twist.linear.x))
    print("Angular speed: "+str(self.twist.angular.z))
    
  def callback(self,data):
    try: 
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    

    frame = cv2.resize(frame, (300,300)) 

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gaussian = cv2.GaussianBlur(hsv,(5,5),0)
    median3 = cv2.medianBlur(hsv,3)
    median5 = cv2.medianBlur(hsv,5)


    lower1 = np.array([0, 0, 200])
    upper1 = np.array([255, 255, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(median5, lower1, upper1)
    mask3 = cv2.inRange(median3, lower1, upper1)
    mask4 = cv2.inRange(gaussian, lower1, upper1)
    # Bitwise-AND mask and original image
    res1 = cv2.bitwise_and(frame,frame, mask= mask)
    res2 = cv2.bitwise_and(frame,frame, mask= mask2)
    res3 = cv2.bitwise_and(frame,frame, mask= mask3)
    res4 = cv2.bitwise_and(frame,frame, mask= mask4)


    cv2.imshow("Original", res1)
    cv2.imshow("Blurred 50", res2)
    cv2.imshow("Blurred 30", res3)
    cv2.imshow("Gaussian", res4)
    cv2.waitKey(2)

    # try:
    #   rospy.Rate(15).sleep() 
    #   self.image_pub.publish(vel_msg)
    # except CvBridgeError as e:
    #   print(e)
 
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
 
def main():
  
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 