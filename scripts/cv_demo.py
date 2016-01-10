#!/usr/bin/env python

import roslib 
#roslib.load_manifest('rbx_vision')
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo():

    def __init__(self):
        self.node_name = "cv_bridget_demo"
        
        rospy.init_node( self.node_name )

        rospy.on_shutdown( self.cleanup )
 
        self.cv_window_name = self.node_name
        #cv2.namedWindow( self.cv_window_name, cv2.WINDOW_AUTOSIZE )

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback )
        rospy.loginfo( "Waiting for image topics..." )

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2( ros_image )
        except CvBridgeError, e:
            print e

        frame = np.array( frame, dtype=np.uint8 )
        display_image = self.process_image( frame )
        cv2.imshow( self.cv_window_name, display_image ) 
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User hit q key to quit.")
        
        #print "FINISHED IMSHOW"

    def process_image( self, frame ):
        grey = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
        grey = cv2.blur( grey, (7,7) )
        edges = cv2.Canny( grey, 15.0, 30.0 )
        return edges

    def cleanup(self):
        print "Shutting down vision node"
        cv2.destroyAllWindows()

def main(args):
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

