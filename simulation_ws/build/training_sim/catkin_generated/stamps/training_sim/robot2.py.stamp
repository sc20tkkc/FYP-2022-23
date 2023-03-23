#!/usr/bin/env python3
# Code taken and adapted from Jaymeson Heller

import rospy
import numpy as np
import time
from sensor_msgs.msg import Range, LaserScan, Image
from random import randrange
from geometry_msgs.msg import Twist

rotate_speed = 2.5
move_speed = 0.1
pub_msg = Twist()
send_msg_ = False

class MoveRobot2:
    def __init__(self):
        # rospy.init_node('Script_controlling_robot2', anonymous=False)
        self.pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(5) #5Hz

def normalise_prox(data):
  total_ran = 0
  rans = 0
  for ran in data.ranges:
    if "inf" not in str(ran) and "-inf" not in str(ran):
      total_ran=total_ran+float(ran)
      rans= rans +1
  if rans==0:
    return 0
  else:
    avg = total_ran/rans
    return int(6- ((round(avg * 20) / 20)*20))

def normalise_line(data):
  total_data = 0
  datas = 0
  highest = 0
  for dats in data.data:
    if int(dats)>highest:
      highest = int(dats)
  if highest>60:
    return 1#1 means that the line has been detected
  else: 
    return 0 # 0 means that there's no line detected

def backward():
  move.linear.x = move_speed*1.4
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = 0
  pub.publish(move)

def backright():
  move.linear.x = move_speed*1.4
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = -(rotate_speed)
  pub.publish(move)

def backleft():
  move.linear.x = move_speed*1.4
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = (rotate_speed)
  pub.publish(move)

def rotate_left():
  move.linear.x = 0
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = -(rotate_speed)
  pub.publish(move)

def rotate_right():
  move.linear.x = 0
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = rotate_speed
  pub.publish(move)

def forward():
  move.linear.x = -(move_speed)
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = 0
  pub.publish(move)

def stop():
  move.linear.x = 0
  move.linear.y = 0
  move.linear.z = 0
  move.angular.x = 0
  move.angular.y = 0
  move.angular.z = 0
  pub.publish(move)

def stay_in_circle():
  num_turn=0
  back_turn=0
  direction = 1 #1 means right, 0 means left
  face_left = 0
  face_right = 0
  front_turn = 0
  start=1
  runs = 0
  while not rospy.is_shutdown() and runs<300:
    try:

      mid_line_data = rospy.wait_for_message('/robot2/camera/rgb/image_raw', Image, timeout=5)
      mid_line = normalise_line(mid_line_data)

      left_line_data = rospy.wait_for_message('/robot2/camera_left/rgb/image_raw', Image, timeout=5)
      left_line = normalise_line(left_line_data)

      right_line_data = rospy.wait_for_message('/robot2/camera_right/rgb/image_raw', Image, timeout=5)
      right_line = normalise_line(right_line_data)

    except:
      pass
      mid_line = 0
      left_line = 0
      right_line = 0

    if mid_line==1:
      backward()
      back_turn=1
      num_turn=0
      face_left=0
      face_right=0

    elif (left_line==1 and right_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=1
      face_left=0
      face_right=0
      # backward()

    elif (right_line==1 and left_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=0
      face_left=0
      face_right=0

    elif back_turn>0 and back_turn<4:
      backward()
      back_turn+=1

    elif front_turn>0 and front_turn<4:
      forward()
      front_turn+=1
    elif front_turn==4:
      start=1

    elif face_right>0 and face_right<3:
      rotate_right()
      face_right+=1

    elif face_left>0 and face_left<3:
      rotate_left()
      face_left+=1

      
    else:
      back_turn=0
      if direction == 1:
        if num_turn<9:
          rotate_right()
          num_turn+=1
        else:
          num_turn=0
          back_turn=0
          direction = 1 #1 means right, 0 means left
          face_left = 0
          face_right = 0
          front_turn = 0
          start=1

      else:
        if num_turn<9:
          rotate_left()
          num_turn+=1
        else:
          num_turn=0
          back_turn=0
          direction = 1 #1 means right, 0 means left
          face_left = 0
          face_right = 0
          front_turn = 0
          start=1

    try:
      rate.sleep()
    except:
      pass
    runs+=1


def aggressive():
  num_turn=0
  back_turn=0
  direction = 1 #1 means right, 0 means left
  face_left = 0
  face_right = 0
  runs = 0
  while not rospy.is_shutdown() and runs < 300:
    try:

      mid_line_data = rospy.wait_for_message('/robot2/camera/rgb/image_raw', Image, timeout=5)
      mid_line = normalise_line(mid_line_data)

      left_line_data = rospy.wait_for_message('/robot2/camera_left/rgb/image_raw', Image, timeout=5)
      left_line = normalise_line(left_line_data)

      right_line_data = rospy.wait_for_message('/robot2/camera_right/rgb/image_raw', Image, timeout=5)
      right_line = normalise_line(right_line_data)

      front_right_prox_data = rospy.wait_for_message('/robot2/prox/front_right_range_sensor', LaserScan, timeout=5)
      front_right_prox = normalise_prox(front_right_prox_data)

      front_left_prox_data = rospy.wait_for_message('/robot2/prox/front_left_range_sensor', LaserScan, timeout=5)
      front_left_prox = normalise_prox(front_left_prox_data)

      right_prox_data = rospy.wait_for_message('/robot2/prox/right_range_sensor', LaserScan, timeout=5)
      right_prox = normalise_prox(right_prox_data)

      left_prox_data = rospy.wait_for_message('/robot2/prox/left_range_sensor', LaserScan, timeout=5)
      left_prox = normalise_prox(left_prox_data)
    except:
      pass
      mid_line = 0
      left_line = 0
      right_line = 0
      front_right_prox = 0
      front_left_prox = 0
      right_prox = 0
      left_prox = 0


    if mid_line==1:
      backward()
      back_turn=1
      num_turn=0
      face_left=0
      face_right=0

    elif (left_line==1 and right_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=1
      face_left=0
      face_right=0
      # backward()

    elif (right_line==1 and left_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=0
      face_left=0
      face_right=0

    elif back_turn>0 and back_turn<4:
      backward()
      back_turn+=1

    elif face_right>0 and face_right<3:
      rotate_right()
      face_right+=1

    elif face_left>0 and face_left<3:
      rotate_left()
      face_left+=1

      
    else:
      if front_left_prox>0 or front_right_prox>0:
        if front_left_prox>=5 and front_right_prox<=4:
          backward()
          back_turn=2
          face_left = 1
        elif front_right_prox>=5 and front_left_prox<=4:
          backward()
          back_turn=2
          face_right = 1
        else:
          forward()
      elif left_prox >0:
        backward()
        direction=0
        num_turn=0
      elif right_prox >0:
        backward()
        direction=1
        num_turn=0
      back_turn=0
      if direction == 1:
        if num_turn<9:
          rotate_right()
          num_turn+=1
        else:
          forward()
      else:
        if num_turn<9:
          rotate_left()
          num_turn+=1
        else:
          forward()

    try:
      rate.sleep()
    except:
      pass
    runs+=1


def evasive():
  num_turn=0
  back_turn=0
  direction = 1 #1 means right, 0 means left
  face_left = 0
  face_right = 0
  runs = 0
  while not rospy.is_shutdown() and runs < 300:
    try:

      mid_line_data = rospy.wait_for_message('/robot2/camera/rgb/image_raw', Image, timeout=5)
      mid_line = normalise_line(mid_line_data)

      left_line_data = rospy.wait_for_message('/robot2/camera_left/rgb/image_raw', Image, timeout=5)
      left_line = normalise_line(left_line_data)

      right_line_data = rospy.wait_for_message('/robot2/camera_right/rgb/image_raw', Image, timeout=5)
      right_line = normalise_line(right_line_data)

      front_right_prox_data = rospy.wait_for_message('/robot2/prox/front_right_range_sensor', LaserScan, timeout=5)
      front_right_prox = normalise_prox(front_right_prox_data)

      front_left_prox_data = rospy.wait_for_message('/robot2/prox/front_left_range_sensor', LaserScan, timeout=5)
      front_left_prox = normalise_prox(front_left_prox_data)

      right_prox_data = rospy.wait_for_message('/robot2/prox/right_range_sensor', LaserScan, timeout=5)
      right_prox = normalise_prox(right_prox_data)

      left_prox_data = rospy.wait_for_message('/robot2/prox/left_range_sensor', LaserScan, timeout=5)
      left_prox = normalise_prox(left_prox_data)
    except:
      pass
      mid_line = 0
      left_line = 0
      right_line = 0
      front_right_prox = 0
      front_left_prox = 0
      right_prox = 0
      left_prox = 0


    if mid_line==1:
      backward()
      back_turn=1
      num_turn=0
      face_left=0
      face_right=0

    elif (left_line==1 and right_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=1
      face_left=0
      face_right=0

    elif (right_line==1 and left_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=0
      face_left=0
      face_right=0

    elif back_turn>0 and back_turn<4:
      backward()
      back_turn+=1

    elif face_right>0 and face_right<3:
      rotate_right()
      face_right+=1

    elif face_left>0 and face_left<3:
      rotate_left()
      face_left+=1

      
    else:
      if front_left_prox>0 or front_right_prox>0:
        if (front_left_prox - front_right_prox>=3) or front_right_prox==0:
          rotate_right()
          num_turn = 1
          direction = 1
        elif (front_right_prox - front_left_prox>=3) or front_left_prox==0:
          rotate_left()
          num_turn = 1
          direction = 0
        else:
          rotate_right()
          face_right=1
          direction = 0
      elif left_prox>3:
        direction = 1
        forward()
      elif right_prox>3:
        direction=0
        forward()
      back_turn=0
      if direction == 1:
        if num_turn<9:
          rotate_right()
          num_turn+=1
        else:
          forward()
      else:
        if num_turn<9:
          rotate_left()
          num_turn+=1
        else:
          forward()

    try:
      rate.sleep()
    except:
      pass
    runs+=1


def wide_circle():
  num_turn=0
  back_turn=0
  direction = 1 #1 means right, 0 means left
  face_left = 0
  face_right = 0
  start = 1
  front_turn = 0
  runs = 0
  while not rospy.is_shutdown() and runs>300:
    try:

      mid_line_data = rospy.wait_for_message('/robot2/camera/rgb/image_raw', Image, timeout=5)
      mid_line = normalise_line(mid_line_data)

      left_line_data = rospy.wait_for_message('/robot2/camera_left/rgb/image_raw', Image, timeout=5)
      left_line = normalise_line(left_line_data)

      right_line_data = rospy.wait_for_message('/robot2/camera_right/rgb/image_raw', Image, timeout=5)
      right_line = normalise_line(right_line_data)

    except:
      pass
      mid_line = 0
      left_line = 0
      right_line = 0
      break

    if mid_line==1:
      backward()
      back_turn=1
      num_turn=0
      face_left=0
      face_right=0

    elif (left_line==1 and right_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=1
      face_left=0
      face_right=0

    elif (right_line==1 and left_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=0
      face_left=0
      face_right=0

    elif back_turn>0 and back_turn<4:
      backward()
      back_turn+=1
    elif front_turn>0 and front_turn<4:
      forward()
      front_turn+=1
    elif front_turn==4:
      start=1

    elif face_right>0 and face_right<3:
      rotate_right()
      face_right+=1

    elif face_left>0 and face_left<3:
      rotate_left()
      face_left+=1

      
    else:
      back_turn=0
      if direction == 1:
        if num_turn<17:
          rotate_right()
          num_turn+=1
        else:
          forward()
          
      else:
        if num_turn<17:
          rotate_left()
          num_turn+=1
        else:
          forward()

    try:
      rate.sleep()
    except:
      pass
    runs+=1


def evasive_agression():
  num_turn=0
  back_turn=0
  direction = 1 #1 means right, 0 means left
  face_left = 0
  face_right = 0
  push = 0
  runs = 0
  while not rospy.is_shutdown() and runs < 300:
    try:

      mid_line_data = rospy.wait_for_message('/robot2/camera/rgb/image_raw', Image, timeout=5)
      mid_line = normalise_line(mid_line_data)

      left_line_data = rospy.wait_for_message('/robot2/camera_left/rgb/image_raw', Image, timeout=5)
      left_line = normalise_line(left_line_data)

      right_line_data = rospy.wait_for_message('/robot2/camera_right/rgb/image_raw', Image, timeout=5)
      right_line = normalise_line(right_line_data)

      front_right_prox_data = rospy.wait_for_message('/robot2/prox/front_right_range_sensor', LaserScan, timeout=5)
      front_right_prox = normalise_prox(front_right_prox_data)

      front_left_prox_data = rospy.wait_for_message('/robot2/prox/front_left_range_sensor', LaserScan, timeout=5)
      front_left_prox = normalise_prox(front_left_prox_data)

      right_prox_data = rospy.wait_for_message('/robot2/prox/right_range_sensor', LaserScan, timeout=5)
      right_prox = normalise_prox(right_prox_data)

      left_prox_data = rospy.wait_for_message('/robot2/prox/left_range_sensor', LaserScan, timeout=5)
      left_prox = normalise_prox(left_prox_data)
    except:
      pass
      mid_line = 0
      left_line = 0
      right_line = 0
      front_right_prox = 0
      front_left_prox = 0
      right_prox = 0
      left_prox = 0


    if mid_line==1:
      backward()
      back_turn=1
      num_turn=0
      face_left=0
      face_right=0

    elif (left_line==1 and right_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=1
      face_left=0
      face_right=0

    elif (right_line==1 and left_line==0):
      backward()
      back_turn=1
      num_turn=0
      direction=0
      face_left=0
      face_right=0

    elif back_turn>0 and back_turn<4:
      backward()
      back_turn+=1

    elif face_right>0 and face_right<3:
      rotate_right()
      face_right+=1

    elif face_left>0 and face_left<3:
      rotate_left()
      face_left+=1

      
    else:
      if front_left_prox>0 or front_right_prox>0:
        if front_left_prox>=5 and front_right_prox<=4:
          backward()
          back_turn=2
          face_left = 1
        elif front_right_prox>=5 and front_left_prox<=4:
          backward()
          back_turn=2
          face_right = 1
        elif  push<4:
          forward()
          push +=1
        else:
          backward()
          back_turn=0
          face_right = 1
          push= 0

      elif left_prox >3:
        backward()
        direction=0
      elif right_prox >3:
        backward()
        direction=1
      back_turn=0
      if direction == 1:
        if num_turn<9:
          rotate_right()
          num_turn+=1
        else:
          forward()
      else:
        if num_turn<9:
          rotate_left()
          num_turn+=1
        else:
          forward()

    try:
      rate.sleep()
    except:
      pass
    runs+=1

def stationary():
  runs = 0
  while not rospy.is_shutdown() and runs < 300:
    rotate_right()

    try:
      rate.sleep()
    except:
      pass
    runs+=1




def main():
  while 1==1:
    rospy.on_shutdown(stop)
    choice = randrange(6)
    if choice==0:
      stay_in_circle()
    elif choice==1:
      aggressive()
    elif choice==2:
      evasive()
    elif choice ==3:
      wide_circle()
    elif choice==4:
      evasive_agression()
    else:
      stationary()


if __name__ == '__main__':
    rospy.init_node('robot2_control', anonymous=False)
    rate = rospy.Rate(20)
    move = Twist()
    pub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size = 1)
    main()