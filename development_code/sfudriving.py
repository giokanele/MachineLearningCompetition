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

 



state =5 # 0=left, 1=forwardy, 2=right, 5 = no initialized
prevx = 0
prevz = 0

class image_converter:

  def __init__(self):
    self.sfu_model = keras.models.load_model("/home/fizzer/ros_ws/src/my_controller/secondSfuModel.h5")
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
    cv2.imshow("Original", frame)
    
    frame = cv2.resize(frame[400:], (200,200)) 


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
    goto_state = self.sfu_model.predict(np.expand_dims(np.array([vertical_blur]), axis = -1))


    

    softmax_output = np.exp(goto_state[0]) / np.sum(np.exp(goto_state[0]))


    print(goto_state)

    speed_factor = 2

    x = speed_factor*(goto_state[0][1]-0.5*goto_state[0][0]-0.5*goto_state[0][2])
    
    z = (3*(goto_state[0][0]-goto_state[0][2])/softmax_output[0])

    if x<0.05:
        x = 0.05
    elif x>0.5:
        x = 0.5
    else:
        x = np.log(np.log(x+1)+1)
    if  z<-2:
        z = -2 
    elif z>2:
        z = 2
    
    
    global prevx, prevz
    self.twist.linear.x = 0.75*x + 0.25*prevx
    self.twist.angular.z = 0.75*z + 0.25*prevz
    prevx = self.twist.linear.x
    prevz = self.twist.angular.z


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