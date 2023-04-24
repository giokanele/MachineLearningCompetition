#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import random
import threading

from keras import models
import time


holder = -2
index = 0
user_input = ""
capture = threading.Event()
capture.clear()
reset = threading.Event()
licensePlateModel = models.load_model("/home/fizzer/ros_ws/src/my_controller/licenseNNv2.keras")

class history:

  def __init__(self):
    self.plates = []
    self.state = 0
    self.hist = list(np.zeros(50))
    self.green = False
    self.passed = False
    self.results = []
    self.sleep = 0
    self.timings = {1:(15, 30), 2:(15, 30),3:(15, 30), 4:(15, 30),5:(15, 30), 6:(15, 30),7:(15, 30), 8:(15, 30)}
    self.license_pub = rospy.Publisher("/license_plate",String, queue_size=1)

  def input(self, found, plate = "YOLO"):
    self.hist.append(found)
    send = False
    if(self.sleep > 0):
      self.sleep -= 1
      return
    if found == 1:
      self.plates.append(plate)

    if(self.state < 8):
      sample, waittime = self.timings[self.state + 1]
    else:
      sample, waittime = (20, 30)

    if(np.average(self.hist[-sample:]) > 0.7):
      self.green = True
    else:
      if self.green == True:
        send = True
        self.state += 1
      self.green = False
      
    if(send == True):
      self.sendmessage()
      # print("Sending!")
      self.sleep = waittime
      send = False

    # print("found:", found, "State is:", self.state, "Green:", self.green, "hist_avg:", np.average(self.hist[-20:]), "Results", self.results)
    return

  def sendmessage(self):
    List = self.plates[max(0, len(self.plates) - 10):]
    plate = max(set(List), key = List.count)
    self.results.append(plate)
    #reset variables
    self.plates.clear()
    self.hist = list(np.zeros(50))
    self.license_pub.publish("Gio's team,password,{},".format(self.state) + plate)
    return

class image_converter:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/R1/cmd_vel",Twist, queue_size=1)
    self.license_pub = rospy.Publisher("/license_plate",String, queue_size=1)

    self.twist = Twist()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.history = history()

    time.sleep(1)
    print("Message sent!")
    self.license_pub.publish("Gio's team,password,0,AA00")
    

  def callback(self,data):
    global capture
    global user_input
    global reset
    if reset.is_set():
      self.history.__init__()
      print("Reset successful!")
      self.license_pub.publish("Gio's team,password,0,AA00")
      reset.clear()

    
    timer = time.time()

    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    frame2 = np.copy(frame)
    ## size of picture: (x: 1280, y: 720)
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)


    # Threshold the HSV image to get only blue colors
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    blue = cv2.inRange(hsv, lower_blue, upper_blue)
    blue[0:330] = 0
    contours3, _ = cv2.findContours(blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours3 = sorted(contours3, key=cv2.contourArea, reverse=True)
    contours3 = contours3[:min(len(contours3), 3)]
    blue_lines = np.zeros_like(blue)
    for contour in contours3:
      cv2.drawContours(blue_lines, [contour], 0, 255, 3)


    lower_grey = np.array([0,0, 100])
    upper_grey = np.array([255,30,230])
    grey = cv2.inRange(hsv, lower_grey, upper_grey)
    grey[0:330] = 0
    contours, hierarchy = cv2.findContours(grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contours = contours[: min(10, len(contours))]

    #grey_lines = np.zeros_like(blue)
    #for contour in contours:
    #  cv2.drawContours(grey_lines, [contour], 0, 255, 3)
    #comb = cv2.bitwise_and(blue_lines, grey_lines)
    #cv2.imshow("comb", comb)

    square_contours = []
    
    for contour in contours:
      grey_line = np.zeros_like(blue_lines)
      cv2.drawContours(grey_line, [contour], 0, 255, 20)
      andd = cv2.bitwise_and(blue_lines, grey_line)
      M = cv2.moments( andd )
      if M["m00"] == 0: #check if contour intersects blue contours
        continue
      if cv2.contourArea(contour) == 0:
        continue
      if cv2.arcLength(contour, True) / cv2.contourArea(contour) > 0.3: #remove long contours
        continue
      approx = cv2.approxPolyDP(contour, 0.13 * cv2.arcLength(contour, True), True)
      if len(approx) > 6:
        continue
      if getInfo(contour, "maxy") >= 715: #remove contours touching the bottom of page
        continue
      square_contours.append(contour)
      cv2.drawContours(frame, [contour], 0, (0, 255, 0), 3)
    
    if len(square_contours) == 0:
      found = False
    else:
      found = True

      x, y = ((getInfo(square_contours[0], "maxx") + getInfo(square_contours[0], "minx"))//2, 
                 (getInfo(square_contours[0], "maxy") + 3*getInfo(square_contours[0], "miny"))//4 )
      
      cv2.drawContours(frame, square_contours, 0, (255, 0, 255), 3)

      cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
      colour = blur[y, x]
      topBox = square_contours[0]
      #dark shading - mask split in 2
      #get coordinates of lower box
      #and circle them
      if np.linalg.norm( colour - np.array((100, 100, 100)) ) < 10: 
        #already in two chunks
        
        corners = getInfo(topBox, attr = "corners")
        max0 = getInfo(topBox,  attr = "maxx")
        min0 = getInfo(topBox, attr = "minx")
        maxy0 = getInfo(topBox, attr = "maxy")

        found = False
        for i in range(1, len(square_contours)):
          if abs(getInfo(square_contours[i], attr = "maxx") - max0) < 25 \
                      and abs(getInfo(square_contours[i], attr = "minx") - min0) < 25 \
                      and getInfo(square_contours[i], attr = "miny") > maxy0 - 15:
            bottomBox = square_contours[i]
            corners = getInfo(square_contours[i], attr = "corners")
            found = True
            cv2.drawContours(frame, [topBox, bottomBox], -1, (0, 255, 255), 3)
            # print( i, getInfo(square_contours[i], attr = "minx") )
            break

      #light shading
      #license plate is a subset of topBox
      #get license plate coordinates
      elif np.linalg.norm( colour - np.array((200, 200, 200)) ) < 10 or np.linalg.norm( colour - np.array((122, 122, 122)) ) < 10:
        # cv2.drawContours(frame, [topBox], 0, (0, 0, 255), 3)
        found = False
        blank = np.zeros_like(frame)
        zone = cv2.drawContours(blank, [topBox], -1, (255, 255, 255), -1)
        newImage = cv2.bitwise_and(blur, zone)

        points = np.array( getInfo(topBox, attr = "corners") ).reshape(-1, 1, 2)
        cv2.fillPoly(blank, [points], (255, 255, 255))
        
        # print(newImage[, y])
        grey = cv2.inRange(newImage, colour - 10, colour + 10)
        contours2, _ = cv2.findContours(grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours2 = sorted(contours2, key=cv2.contourArea, reverse=True)
        cv2.drawContours(newImage, contours2, -1, (0, 255, 0), 3)

        if len(contours2) != 0:
          topBox = contours2[0]
          maxy = getInfo(topBox, attr = "maxy")
          found = False
          for i in range(1, len(contours2)):
            if getInfo(contours2[i], attr = "maxy") > maxy:
              bottomBox = contours2[i]
              found = True
              corners = getInfo(bottomBox, attr = "corners")
              for vertex in corners:
                cv2.circle(newImage, vertex, 5, (0, 0, 255), -1)
              break
          
          corners = getInfo(topBox, attr = "corners")
          for vertex in corners:
              cv2.circle(newImage, vertex, 5, (0, 0, 255), -1)
      else:
        #abort mission
        found = False

      if found == True:
      # final license plate part
        license = (  getInfo(topBox, "BL"), getInfo(topBox, "BR"), getInfo(bottomBox, "TR"), getInfo(bottomBox, "TL")  )
        for vertex in license:
          cv2.circle(frame, vertex, 5, (255, 0, 0), -1)
        licensePlate = np.zeros_like(frame, shape = (100, 450, 3))
        rows,cols = (100, 450)
        
        # define four points on input image
        pts1 = np.float32(license)
        # define the corresponding four points on output image
        pts2 = np.float32([[0,0],[449,0],[449,99],[0,99]])
        # get the perspective transform matrix
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        licensePlate = cv2.warpPerspective(frame2,matrix,(cols, rows))

        slice = {0:27, 1:103, 2:255, 3:332}
        ymin = 30
        width = 85
        height = 60
        for i in range(4):
          cv2.rectangle(licensePlate, (slice[i], ymin), (slice[i] + width, ymin + height), (0,255,0), 2)
        cv2.imshow("plate", licensePlate)
        

        pic = licensePlate*(100/np.average(licensePlate[:, :, 2])) #normalize
        ans = ""
        global licensePlateModel
        for i in range(4):
          character = pic[ymin:ymin + height, slice[i]:slice[i] + width, 2]
          y_predict = licensePlateModel(np.expand_dims(character, axis = 0))[0]
          if i == 0 or i == 1:
            ans += "ABCDEFGHIJKLMNOPQRSTUVWXYZ"[np.argmax(y_predict[0:26])]
          else:
            ans += "0123456789"[np.argmax(y_predict[26:])]

        cv2.putText(frame,ans, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))

        # Save image if instructed - for data collection
        if capture.is_set():
          number = random.randint(1000,9999) # prevent duplicate names
          filename = "plate" + "_" + user_input + "_" + str(number) + ".png" 
          cv2.imwrite("plateData2/" + filename, licensePlate)
          print("Image Saved:", filename)
        
        self.history.input(1, plate = ans)
    
    if(found == False):
      self.history.input(0)
    
    change = time.time() - timer
    # print(change)
    
    cv2.imshow("Image window", frame)
    capture.clear()
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

def getInfo(contour, attr = "None"):
  if attr == "com": 
    M = cv2.moments(contour)
    if M["m00"] != 0:
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
    else: 
      cX = 0
      cY = 0
    return (cX, cY)
  if attr == "area":
    return cv2.contourArea(contour)
  if attr == "maxx":
    return np.max( contour[:,0, 0] )
  if attr == "maxy":
    return np.max( contour[:,0, 1] )
  if attr == "minx":
    return np.min( contour[:,0, 0] )
  if attr == "miny":
    return np.min( contour[:,0, 1] )
  if attr == "BR":
    a = np.argmax( contour[:,0, 0] + contour[:,0, 1] )
    return contour[a, 0]
  if attr == "TL":
    a = np.argmin( contour[:,0, 0] + contour[:,0, 1] )
    return contour[a, 0]
  if attr == "TR":
    a = np.argmax( contour[:,0, 0] - contour[:,0, 1] )
    return contour[a, 0]
  if attr == "BL":
    a = np.argmin( contour[:,0, 0] - contour[:,0, 1] )
    return contour[a, 0]
  if attr == "corners": 
    return (getInfo(contour, attr = "TL"), getInfo(contour, attr = "TR"), getInfo(contour, attr = "BR"), getInfo(contour, attr = "BL"))
  return 
 
def main():
  
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

def collect_plates():
  global user_input
  
  while True:
    value = input("Enter something: ")
    if(value == "q"):
      capture.set()
      print("Message sent")
    else: 
      user_input = value
      # Do something with the user input
def reset_button():
  global reset
  while True:
    value = input()
    if(value == "reset"):
      reset.set()
      print("Resetting queue")


if __name__ == '__main__':
    if False: # for data collection
      background_thread = threading.Thread(target=collect_plates)
      background_thread.daemon = True
      background_thread.start()
    if True:
      background_thread = threading.Thread(target=reset_button)
      background_thread.daemon = True
      background_thread.start()
      reset.clear()

    main()