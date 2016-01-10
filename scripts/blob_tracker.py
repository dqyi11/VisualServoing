#!/usr/bin/env python

import roslib 
#roslib.load_manifest('rbx_vision')
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class BlobTracker():

    def __init__(self):
        self.node_name = "blob_tracker"
        
        rospy.init_node( self.node_name )

        rospy.on_shutdown( self.cleanup )
 
        self.cv_window_name = self.node_name
        #cv2.namedWindow( self.cv_window_name, cv2.WINDOW_AUTOSIZE )

        self.lower_red_l = np.array([0,50,50])
        self.lower_red_h = np.array([10,255,255])
        self.upper_red_l = np.array([170,50,50])
        self.upper_red_h = np.array([180,255,255])
 
        self.kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (8,8) )
 
        self.last_x = -1
        self.last_y = -1

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback )
        rospy.loginfo( "Waiting for image topics..." )

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2( ros_image, "bgr8" )
        except CvBridgeError, e:
            print e

        pos_x, pos_y = self.find_position( frame )
        
        if pos_x >= 0 and pos_y >= 0:
            cv2.circle( frame, (pos_x, pos_y), 10, (0,255,0), 2 )
        self.last_x = pos_x
        self.last_y = pos_y
        cv2.imshow( self.cv_window_name, frame ) 
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            rospy.signal_shutdown("User hit q key to quit.")
        

    def find_position ( self, frame ):
        pos_x = self.last_x
        pos_y = self.last_y

        hsv_img = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV )
        mask1 = cv2.inRange( hsv_img, self.lower_red_l, self.lower_red_h )
        mask2 = cv2.inRange( hsv_img, self.upper_red_l, self.upper_red_h )
       
        mask = mask1 + mask2       

        mask = cv2.erode( mask, self.kernel )
        mask = cv2.dilate( mask, self.kernel )

        mask = cv2.dilate( mask, self.kernel ) 
        mask = cv2.erode( mask, self.kernel )

        o_moments = cv2.moments( mask )
        d_m01 = o_moments['m01']
        d_m10 = o_moments['m10']
        d_area = o_moments['m00']
        print "MOMENTS " + str(d_m01) + " " + str(d_m10) + " " + str(d_area)

        if d_area > 10000:
            pos_x = int(d_m10 / d_area)
            pos_y = int(d_m01 / d_area)
            print "[" + str(pos_x) + " , " + str(pos_y) + "]"
        return pos_x, pos_y 

    def cleanup(self):
        print "Shutting down vision node"
        cv2.destroyAllWindows()

def main(args):
    try:
        BlobTracker()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

