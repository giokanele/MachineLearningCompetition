#!/usr/bin/env python3

import rospy
import cv2
import sys
import os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import threading



from tensorflow import keras

 


state =5 # 0=left, 1=forward, 2=right, 5 = no initialized
prevx = 0
prevz = 0
competition_state = 0 # 0 = hard coded turn, 1 = driving outer ring, 2 = go through crosswalk
frames = 0
oncrosswalk = False
crosswalks_crossed = 0
flag  = False
sfu_model = keras.models.load_model("/home/fizzer/ros_ws/src/my_controller/secondSfuModel.h5")
road_model = keras.models.load_model("/home/fizzer/ros_ws/src/my_controller/thirdModel.h5")

class image_converter:

  def __init__(self):
    
    self.cmd_vel_pub = rospy.Publisher("/R1/cmd_vel",Twist, queue_size=1)
    self.license_pub = rospy.Publisher("/license_plate",String, queue_size=1)

    self.twist = Twist()


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        
        

  def callback(self,data):
    global prevx, prevz, frames, competition_state, oncrosswalk, crosswalks_crossed, road_model, sfu_model,flag
    try: 
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow("Original", frame)
    
    #HARD CODE FOR FIRST TURN
    if competition_state == 0:
       
       self.twist.angular.z = 1.8
       self.twist.linear.x = 0.5
       self.cmd_vel_pub.publish(self.twist)
       print("hard code turn")
       print(frames)
       frames += 1

       if frames >19:
          competition_state = 1
          self.twist.angular.z = 0
          self.twist.linear.x = 0
          self.cmd_vel_pub.publish(self.twist)
          frames = 0

    img = frame
       
    
    frame = cv2.resize(frame[400:], (200,200))



    #OUTER RING
    #
    #
    if competition_state == 1:

      #*** ROAD IMAGE PROCESSING ***
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

      lower1 = np.array([0, 0, 157])
      upper1 = np.array([0, 0, 255])
      # Threshold the HSV image to get only blue colors
      mask = cv2.inRange(hsv, lower1, upper1)
    
      # Bitwise-AND mask and original image
      res = cv2.bitwise_and(frame,frame, mask= mask)
      bgr = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
      grayscale = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
      
      #*** END ROAD IMAGE PROCESSING ***

      #*** ROAD STATE PREDICTION ***
      goto_state = road_model(np.expand_dims(np.array([grayscale]), axis = -1))
      softmax_output = np.exp(goto_state[0]) / np.sum(np.exp(goto_state[0]))

      x = (goto_state[0][1]-0.1*goto_state[0][0]-0.1*goto_state[0][2])/(100*(softmax_output[0]+softmax_output[2]))
    
      z = 0.2*(goto_state[0][0]-goto_state[0][2])/softmax_output[1]

      max_ang_vel = 1.2
      max_lin_vel = 0.4
      if x<0:
          x = 0.1
      elif x**(1/4)>max_lin_vel:
          x = max_lin_vel
      else:
          x = (x)**(1/4)
      if  z<-max_ang_vel:
          z = -max_ang_vel
      elif z>max_ang_vel:
          z = max_ang_vel
      
      
      self.twist.linear.x = x
      self.twist.angular.z = z
      
      self.cmd_vel_pub.publish(self.twist)

      cv2.imshow("img being processed", grayscale)

      #full image blur, mask for red cross walk, find contours, find largest contour, check if bottom edge is touching bottom of image
      img = cv2.GaussianBlur(img, (7, 7), 0)
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      redlower = np.array([0, 157, 44])
      redupper = np.array([39, 255, 255])
      redmask = cv2.inRange(hsv, redlower, redupper)
      cv2.imshow('redmask',redmask)
      test1 = cv2.cvtColor(redmask, cv2.COLOR_GRAY2BGR)
      contours, _ = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      cont_len = contours.__len__()
      if contours:
        contourtest, _ = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(test1, contourtest, -1, (255, 0, 255), 2)

        largest_contour = max(contours, key=cv2.contourArea)
        # Get bounding box coordinates of largest contour
        xcoord1, ycoord1, wcoord1, hcoord1 = cv2.boundingRect(largest_contour)
        # Check if the bottom edge of the bounding box is touching the bottom of the image
        if ycoord1 + hcoord1 +20 >  img.shape[0] and cv2.contourArea(largest_contour) > 50 and cont_len ==2 and not oncrosswalk:
          print('The largest contour is touching the bottom of the image')
          print("SPEED UP")
          oncrosswalk = True
          crosswalks_crossed += 1
          competition_state = 2
        else:
          print("drive normally")
      else:
         oncrosswalk = False
      if oncrosswalk:
        print("on crosswalk")
        
      else:
        print("not on crosswalk")
      print(crosswalks_crossed)
      if crosswalks_crossed == 2:
         competition_state = 2
         

    #CROSS THE CROSSWALK
    #
    #
    if competition_state == 2:
      img = cv2.GaussianBlur(img[200:500], (7, 7), 0)
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      redlower = np.array([0, 157, 44])
      redupper = np.array([39, 255, 255])
      redmask = cv2.inRange(hsv, redlower, redupper)
      cv2.imshow('redmask',redmask)
      if frames == 0:
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        
        frames += 1

      test1 = cv2.cvtColor(redmask, cv2.COLOR_GRAY2BGR)
      contours, _ = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      cont_len = contours.__len__()
      if contours and frames <= 10:
        c = max(contours, key=cv2.contourArea)
        xcoord, ycoord, wcoord, hcoord = cv2.boundingRect(c)
        error = img.shape[1]/2 - (xcoord + wcoord/2)
        self.twist.angular.z = error/25 
        frames += 1  
        print(error)
      
      if frames > 10 and frames < 20:
        self.twist.linear.x = 1.2
        self.twist.angular.z = 0.0
        frames += 1

      if frames >= 20:
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        frames = 0
        if crosswalks_crossed == 1:
          competition_state = 1
        if crosswalks_crossed ==2:
          competition_state = 3
        
        

      # if frames <20 and frames > 10:
      #   self.twist.angular.z = 0
        # self.twist.linear.x = 1.0

          
          
         


      self.cmd_vel_pub.publish(self.twist)

      

      # # frames +=1
      # # if frames >15:
      # #     competition_state = 3
      # #     frames = 0
            
       



    if competition_state == 3:
      print("competition state 3")
      #*** SFU IMAGE PROCESSING ***
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

      darklower = np.array([16, 39, 89])
      darkupper = np.array([35, 71, 151])

      darkmask = cv2.inRange(hsv, darklower, darkupper)

      lightlower = np.array([12, 32, 153])
      lightupper = np.array([49, 64, 255])

      lightmask = cv2.inRange(hsv, lightlower, lightupper)

      outputtest = cv2.bitwise_or(lightmask,darkmask)

      vertical_blur = cv2.blur(outputtest, (2, 20)) #already in gray scale
      #*** END SFU IMAGE PROCESSING ***
      #*** SFU STATE PREDICTION ***
      goto_state = sfu_model(np.expand_dims(np.array([vertical_blur]), axis = -1))
      
      softmax_output = np.exp(goto_state[0]) / np.sum(np.exp(goto_state[0]))

      x = (goto_state[0][1]-0.1*goto_state[0][0]-0.1*goto_state[0][2])/(100*(softmax_output[0]+softmax_output[2]))
    
      z = 0.2*(goto_state[0][0]-goto_state[0][2])/softmax_output[1]

      max_ang_vel = 1.2
      max_lin_vel = 0.4
      if x<0:
          x = 0.1
      elif x**(1/4)>max_lin_vel:
          x = max_lin_vel
      else:
          x = (x)**(1/4)
      if  z<-max_ang_vel:
          z = -max_ang_vel
      elif z>max_ang_vel:
          z = max_ang_vel
      
      
      self.twist.linear.x = x
      self.twist.angular.z = z
      
      self.cmd_vel_pub.publish(self.twist)

    
    
    cv2.waitKey(2)

    
    
    
    # Do something continuously
   


    
   
 
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