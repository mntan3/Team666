#!/usr/bin/env python
import cv2
import rospy
import time
from std_msgs.msg import *
from sensor_msgs.msg import *
from object_detection.msg import BlobDetections
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
from ackermann_msgs.msg import *
class DriveControllerNode:
    def __init__(self):
        self.previous_x_error = 0
        self.previous_time = time.time()
        self.current_riemann_sum = 0
        self.header = std_msgs.msg.Header()
        self.drive_speed = 1
        self.ableToDrive = True
        self.image_center = 640
        self.node_name = "drive_controller"
        self.isGreen = True
        self.achievedFirstGoal = False
        self.achievedSecondGoal = False
        self.sub_blob = rospy.Subscriber("blob_detections", BlobDetections, self.get_target_cb)
        self.sub_control = rospy.Subscriber("/vesc/joy", Joy, self.drive_control_cb)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped,queue_size=1)
        self.pub_nextGoal = rospy.Publisher("/transition", String, queue_size = 2)
        rospy.loginfo("initialized")

    def drive_control_cb(self, msg):
        if msg.buttons[0] == 1:
            self.ableToDrive == False
        else:
            return

    def drive_control(self, height, x, y):

        drivemsg = AckermannDriveStamped()
        dist = x - 640
        x_error = dist

        K_Prop = 0.0005
        K_Deriv = 0.0
        K_Int = 0.0    
        if self.ableToDrive:
            if not self.achievedFirstGoal:
                if height < 120:
                    drivemsg.drive.speed = 3.0
                else:
                    drivemsg.drive.speed = 0.0
                    self.achievedFirstGoal = True
                current_time = time.time()
                delta_t = current_time - self.previous_time
                delta_x = x_error - self.previous_x_error
                average_x = (x_error + self.previous_x_error)/2

                # using the change in error over the change in time to calculate the rate of change.

       	        deriv_x_error = delta_x/delta_t

                # using the riemann sum to calculate the integral

                int_x_error = self.current_riemann_sum + delta_t * average_x

                drivemsg.drive.steering_angle = (K_Prop * x_error + K_Deriv * deriv_x_error + K_Int * int_x_error) * (-1)
		       
                self.pub_drive.publish(drivemsg)
                self.previous_x_error = x_error
                self.previous_time = current_time
                self.current_riemann_sum = int_x_error

            else:
                print(self.isGreen) 
                if self.isGreen:
                    wall_publisher = "Left"
                    drivemsg.drive.steering_angle = -1.0
                else:
                    wall_publisher = "Right"
                    drivemsg.drive.steering_angle = 1.0
                drivemsg.drive.speed = 3.0
                if not self.achievedSecondGoal:
                    x = 1
                    while x < 10:
                        self.pub_drive.publish(drivemsg)
                        rospy.sleep(0.15)
                        x = x + 1
                    self.pub_nextGoal.publish(wall_publisher)
                    self.achievedSecondGoal = True
        else:
            pub_drive.publish(AckermannDriveStamped(self.header, AckermannDrive(steering_angle = 0.0, speed = 0.0)))
	
    def get_target_cb(self, msg):
        for i in range(0, len(msg.colors)):
            if msg.colors[i].g == 255 or msg.colors[i].r == 255.0:
                if msg.colors[i].g == 255.0:
                    isGreen = True
                if msg.colors[i].r == 255.0:
                    isGreen = False
                print(msg.colors[i].g)
                print(msg.colors[i].r)
                height = msg.sizes[i].data
                center_x, center_y = msg.locations[i].x,msg.locations[i].y
                self.drive_control(height, center_x, center_y)

if __name__ == "__main__":
    rospy.init_node("drive_controller")
    node = DriveControllerNode()
    rospy.spin()

