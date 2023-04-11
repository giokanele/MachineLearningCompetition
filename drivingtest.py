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

 
# boolean to send message between threads
capture = threading.Event()
init = True
myData = []

state =5 # 0=left, 1=forward, 2=right, 5 = no initialized

class image_converter:

  def __init__(self):
    self.road_model = keras.models.load_model("/home/fizzer/ros_ws/src/my_controller/secondModel.h5")
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
    
    
    frame = cv2.resize(frame[400:], (200,200)) 

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0, 0, 157])
    upper1 = np.array([0, 0, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower1, upper1)
   
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    bgr = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
    grayscale = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    goto_state = self.road_model.predict(np.expand_dims(np.array([grayscale]), axis = -1))

    softmax_output = np.exp(goto_state[0]) / np.sum(np.exp(goto_state[0]))


    print(goto_state)

    x = (goto_state[0][1]-goto_state[0][0]-goto_state[0][2])/3
    
    z = (goto_state[0][0]-goto_state[0][2])/softmax_output[0]

    if x<0:
        x = 0
    elif x>0.2:
        x = 0.2
    if  z<-0.2:
        z = -0.2  
    elif z>0.2:
        z = 0.2
    
    self.twist.linear.x = x
    self.twist.angular.z = z   


    # if goto_state == 0:
    #   self.twist.linear.x = 0
    #   self.twist.angular.z = 0.2
    # elif goto_state == 1:
    #     self.twist.linear.x = 0.2
    #     self.twist.angular.z = 0
    # elif goto_state == 2:
    #     self.twist.linear.x = 0
    #     self.twist.angular.z = -0.2

    self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("Original", grayscale)
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