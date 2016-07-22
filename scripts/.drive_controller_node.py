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
#previous_y_error = 0
#previous_time = localtime()
#current_riemann_sum = 0
class DriveControllerNode:
    def __init__(self):
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
        error_x = dist
        error_y = height - 640.0

        #K_Prop = 0
        #K_Deriv = 0
        #K_Int = 0

        #global previous_error
        #global previous_y_error
        #global current_riemann_sum

        # If you wanted to use my code you would replace the below code with my commented one.
	    
        if not self.achievedFirstGoal:
            if dist > 80 and height < 140:               #blob is to the right and blob too far
                drivemsg.drive.steering_angle = -0.5
                drivemsg.drive.speed = 0.4
            elif dist < -80 and height < 140:               #blob is to the left and blob too far
                drivemsg.drive.steering_angle = 0.5
                drivemsg.drive.speed = 0.4
            #elif dist < -80 and height > 140:               #blob is to the left and blob is too close or just right
            #    drivemsg.drive.speed = -0.4
            #    drivemsg.drive.steering_angle = 0.0          #FIX THISSSSS
            #elif dist > 80 and height > 140:                #blob is to the right and blob is too close or just right
            #    drivemsg.drive.speed = -0.4
            #    drivemsg.drive.steering_angle = 0.0         #FIX THISSSSSSS
            elif height < 140:                              #too far
                drivemsg.drive.speed = 0.4
            #elif height > 170:                              #too close
            #    drivemsg.drive.speed = -0.4
            else:
                drivemsg.drive.speed = 0.0
                rospy.loginfo("just right")
                self.achievedFirstGoal = True

        elif self.isGreen:
            print("In isGreen")
            self.pub_nextGoal.publish("Right")
            drivemsg.drive.speed = 1.0
            drivemsg.drive.steering_angle = 1.0

        else:
            print("Red")
            self.pub_nextGoal.publish("Left")
            drivemsg.drive.speed = 1.0
            drivemsg.drive.steering_angle = -1.0

        self.pub_drive.publish(drivemsg)

        if not self.ableToDrive:
            pub_drive.publish(AckermannDriveStamped(self.header, AckermannDrive(steering_angle = 0.0, speed = 0.0)))
    	 # This is where we would implement the PID controller instead of bang bang.

    	 #drivemsg.drive.steering_angle = 0

    	 #if height > 670:
                     #drivemsg.drive.speed = -1
                 #elif height < 650:
                     #drivemsg.drive.speed = 1
                 #else:
                     #drivemsg.drive.speed = 0

    	 #self.pub_drive.publish(drivemsg)

    	 ##current_time = localtime()
    	 ##delta_t = current_time - previous_time
    	 ##delta_y = y_error - previous_y_error
    	 ##average_y = (y_error + previous_y_error)/2

    	 ## # using the change in error over the change in time to calculate the rate of change.

       	 ##deriv_y_error = delta_y/delta_t

		 ## # using the riemann sum to calculate the integral

		 ##int_y_error = current_riemann_sum + delta_t * average_y

		 ##drivemsg.drive.speed = K_Prop * y_error + K_Deriv * deriv_y_error + K_Int + int_y_error
		
		 ##previous_y_error = y_error
		 ##previous_time = current_time
		 ##current_riemann_sum = int_y_error

	#else:
	     #pub_drive.publish(AckermannDriveStamped(self.header, AckermannDrive(steering_angle = 0.0, speed = 0.0)))
	
    def get_target_cb(self, msg):
        #print(len(msg.colors))
        #print(len(msg.sizes))
        #print(type(msg.colors))
        #print(type(msg.sizes[0]))
        for i in range(0, len(msg.colors)):
            #print(msg.colors[i].g)
            if msg.colors[i].g == 255 or msg.colors[i].r == 255:
                if msg.colors[i].g == 255:
                    isGreen = True
                elif msg.colors[i].r == 255:
                    isGreen = False
                height = msg.sizes[i].data
                center_x, center_y = msg.locations[i].x,msg.locations[i].y
                self.drive_control(height, center_x, center_y)

if __name__ == "__main__":
    rospy.init_node("drive_controller")
    node = DriveControllerNode()
    rospy.spin()

