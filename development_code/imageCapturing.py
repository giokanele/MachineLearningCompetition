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
import time 
from pynput import keyboard
from datetime import datetime

 
# boolean to send message between threads
capture = threading.Event()
init = True
myData = []

state =5 # 0=left, 1=forward, 2=right, 5 = no initialized
count = 0
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
    global state, capture
    if capture.is_set():
      if self.twist.linear.x == 0:
        if self.twist.angular.z > 0:
          state  = 0 #left
        elif self.twist.angular.z < 0:
          state = 2 #right
        else:
           state = 5
      else:
        state = 1 #straight

        
    
        
      

  def callback(self,data):
    try: 
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    
    frame = cv2.resize(frame[400:], (200,200))
    
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



    


    cv2.imshow("blurred", grayscale)
    cv2.waitKey(2)

    global myData, init, capture, state,count

    if state <2:
      count +=1
    else:
      count = 0
    
    
    # Do something continuously
    if capture.is_set():
        if init == True: #initalize data collection
            myData = []
        
            init = False
        if init == False and state != 5 and count < 3:
            myData.append(np.array([grayscale,state]))  #append data every frame 

    else:
        if init == False:   #if init just turned off, 
            array = np.array(myData)
            print(array.shape)
            
           

            
            cv2.imshow("first", array[0][0])

            now = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            home_dir = os.path.expanduser('~')

            # Construct the file path
            file_path = os.path.join(home_dir, 'ros_ws', 'src', 'my_controller', 'road_data', now + '.npy')
         
            # save numpy array
            np.save(file_path, array)

            init = True
            state = 5
   


    
   
 
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


def background_function():
    # Start the background thread
    # Accept user input from the command prompt
    while True:
        with keyboard.Events() as events:
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('r'):
                if capture.is_set():
                    capture.clear()
                    print("\nStopped")
                    time.sleep(0.2) #wait a bit until next input
                else:
                    print("\nRecording")
                    capture.set()
                    time.sleep(0.2)


if __name__ == '__main__':
    background_thread = threading.Thread(target=background_function)
    background_thread.daemon = True
    background_thread.start()
    capture.clear()
    main()