#!/usr/bin/env python

import roslib 
#roslib.load_manifest('rbx_vision')
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from turtlebot_controller import *

if __name__ == '__main__':

    node_name = "visual_controller"
        
    rospy.init_node( node_name )

    cv_window_name = node_name
    #cv2.namedWindow( cv_window_name, cv2.WINDOW_AUTOSIZE )

    lower_red_l = np.array([0,50,50])
    lower_red_h = np.array([10,255,255])
    upper_red_l = np.array([170,50,50])
    upper_red_h = np.array([180,255,255])

    green_l = np.array([50,100,50])
    green_h = np.array([70,255,255])
 
    kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (4,4) )
 
    last_x = -1
    last_y = -1

    goal_pos = [-1, -1]

    bridge = CvBridge()
    ctrl = TurtlebotController()

    def click_callback( event, x, y, flags, params ):
        if event == cv2.EVENT_LBUTTONDOWN:
            goal_pos[0] = x
            goal_pos[1] = y
            
    def find_green_position ( frame ):
        pos_x = last_x
        pos_y = last_y

        hsv_img = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV )
        green_mask = cv2.inRange( hsv_img, green_l, green_h )
       
        #green_mask = cv2.erode( green_mask, kernel )
        #green_mask = cv2.dilate( green_mask, kernel )

        green_mask = cv2.dilate( green_mask, kernel ) 
        green_mask = cv2.erode( green_mask, kernel )

        o_moments = cv2.moments( green_mask )
        d_m01 = o_moments['m01']
        d_m10 = o_moments['m10']
        d_area = o_moments['m00']
        #print "MOMENTS " + str(d_m01) + " " + str(d_m10) + " " + str(d_area)

        if d_area > 10000:
            pos_x = int(d_m10 / d_area)
            pos_y = int(d_m01 / d_area)
            #print "[" + str(pos_x) + " , " + str(pos_y) + "]"
        return pos_x, pos_y, green_mask 


    def find_red_position ( frame ):
        pos_x = last_x
        pos_y = last_y

        hsv_img = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV )
        red_mask1 = cv2.inRange( hsv_img, lower_red_l, lower_red_h )
        red_mask2 = cv2.inRange( hsv_img, upper_red_l, upper_red_h )
       
        red_mask = red_mask1 + red_mask2       

        red_mask = cv2.erode( red_mask, kernel )
        red_mask = cv2.dilate( red_mask, kernel )

        red_mask = cv2.dilate( red_mask, kernel ) 
        red_mask = cv2.erode( red_mask, kernel )

        o_moments = cv2.moments( red_mask )
        d_m01 = o_moments['m01']
        d_m10 = o_moments['m10']
        d_area = o_moments['m00']
        #print "MOMENTS " + str(d_m01) + " " + str(d_m10) + " " + str(d_area)

        if d_area > 10000:
            pos_x = int(d_m10 / d_area)
            pos_y = int(d_m01 / d_area)
            #print "[" + str(pos_x) + " , " + str(pos_y) + "]"
        return pos_x, pos_y, red_mask

    def image_callback(ros_image):
        try:
            frame = bridge.imgmsg_to_cv2( ros_image, "bgr8" )
        except CvBridgeError, e:
            print e

        red_pos_x, red_pos_y, red_mask = find_red_position( frame )
        green_pos_x, green_pos_y, green_mask = find_green_position( frame )
       
        cv2.imshow( "red", red_mask )
        cv2.imshow( "green", green_mask ) 
        if red_pos_x >= 0 and red_pos_y >= 0:
            cv2.circle( frame, (red_pos_x, red_pos_y), 10, (0,255,0), 2 )
        if green_pos_x >= 0 and green_pos_y >= 0:
            cv2.circle( frame, (green_pos_x, green_pos_y), 10, (0,255,255), 2 )
        last_x = red_pos_x
        last_y = red_pos_y

        #ctrl.goto( goal_pos, [last_x, last_y])

        cv2.namedWindow( cv_window_name, cv2.WINDOW_AUTOSIZE )
        cv2.setMouseCallback( cv_window_name, click_callback, goal_pos )
        cv2.circle( frame, (goal_pos[0], goal_pos[1]), 10, (0,0,255), 2 )
        cv2.imshow( cv_window_name, frame ) 
        
        if cv2.waitKey(500) & 0xFF == ord('q'):
            rospy.signal_shutdown("User hit q key to quit.")
    
    
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback )
    rospy.loginfo( "Waiting for image topics..." )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.DestroyAllWindows()



