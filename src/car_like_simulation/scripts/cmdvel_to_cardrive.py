#!/usr/bin/env python

import roslib; roslib.load_manifest('car_like_simulation')  
import rospy
import math
import thread
from art_msgs.msg import CarDriveStamped
from geometry_msgs.msg import Twist


class Converter:
  def __init__(self):
	self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        self.pub = rospy.Publisher('pilot/drive', CarDriveStamped)


  def cmd_vel_cb(self, msg):
	  CD = CarDriveStamped();
	  CD.control.steering_angle = msg.angular.z;
	  CD.control.speed = msg.linear.x;
	  self.pub.publish(CD);	  
	  

def main():
    rospy.init_node('cmdvel_to_cardrive')
    converter = Converter()
    rospy.spin()

if __name__ == '__main__':
    main()
