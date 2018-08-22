#!/usr/bin/env python
import rospy
from visual import *
import math
import wx

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

rospy.init_node("test")

def processIMU_message(imuMsg):
   
                
    print("======")
    print(imuMsg.pose.pose.orientation.x)  
    print("======")      

#sub = rospy.Subscriber('imu_data', Imu, processIMU_message)
sub = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, processIMU_message)
rospy.spin() 
