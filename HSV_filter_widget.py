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

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    

    frame = cv2.resize(frame, (300,300))
    
    cv2.waitKey(2)

    global holder
    if holder == 5:
      def nothing(x):
        pass

      # Create a window
      cv2.namedWindow('image')

      # create trackbars for color change
      cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
      cv2.createTrackbar('SMin','image',0,255,nothing)
      cv2.createTrackbar('VMin','image',0,255,nothing)
      cv2.createTrackbar('HMax','image',0,179,nothing)
      cv2.createTrackbar('SMax','image',0,255,nothing)
      cv2.createTrackbar('VMax','image',0,255,nothing)

      # Set default value for MAX HSV trackbars.
      cv2.setTrackbarPos('HMax', 'image', 179)
      cv2.setTrackbarPos('SMax', 'image', 255)
      cv2.setTrackbarPos('VMax', 'image', 255)
    
    if holder >=5:
        
      waitTime = 33



      img = frame
      

      # get current positions of all trackbars
      global hMin, sMin, vMin, hMax, sMax, vMax, phMin, psMin, pvMin, phMax, psMax, pvMax
      hMin = cv2.getTrackbarPos('HMin','image')
      sMin = cv2.getTrackbarPos('SMin','image')
      vMin = cv2.getTrackbarPos('VMin','image')

      hMax = cv2.getTrackbarPos('HMax','image')
      sMax = cv2.getTrackbarPos('SMax','image')
      vMax = cv2.getTrackbarPos('VMax','image')

      # Set minimum and max HSV values to display
      lower = np.array([hMin, sMin, vMin])
      upper = np.array([hMax, sMax, vMax])

      # Create HSV Image and threshold into a range.
      img = cv2.GaussianBlur(img, (7, 7), 0)
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower, upper)
      output = cv2.bitwise_and(img,img, mask= mask)

      darklower = np.array([16, 39, 89])
      darkupper = np.array([35, 71, 151])

      darkmask = cv2.inRange(hsv, darklower, darkupper)

      lightlower = np.array([12, 32, 153])
      lightupper = np.array([49, 64, 255])

      lightmask = cv2.inRange(hsv, lightlower, lightupper)

      

      outputtest = cv2.bitwise_or(lightmask,darkmask)
      #print("outputtest shape: " + str(outputtest.shape))
      blurred_output = cv2.GaussianBlur(outputtest, (7, 7), 0)
      cv2.imshow('blurred_output',blurred_output)
      vertical_blur = cv2.blur(outputtest, (2, 20))

      cv2.imshow('vertical_blur',vertical_blur)

      cv2.imshow('outputtest',outputtest)

      # bgr = cv2.cvtColor(outputtest, cv2.COLOR_HSV2BGR)
      # grayscale = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

      # cv2.imshow('grayscale',grayscale)



      # Print if there is a change in HSV value
      if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
          print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
          phMin = hMin
          psMin = sMin
          pvMin = vMin
          phMax = hMax
          psMax = sMax
          pvMax = vMax


      #mask dark part
      #Hmin = 16
      #Smin = 39
      #Vmin = 89

      #Hmax = 35
      #Smax = 71
      #vMax = 151

      #mask light part
      #Hmin = 12
      #Smin = 32
      #Vmin = 153

      #Hmax = 49
      #Smax = 64
      #vMax = 255

      # Display output image
      
      cv2.imshow('image',output)

 
    holder +=1
 
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