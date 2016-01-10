#!/usr/bin/env python

import cv2
import numpy as np

if __name__ == '__main__':
    
    img = 122*np.ones( (800,600,3), dtype=np.uint8 )
    goal_pos = [-1, -1]
    
    def click_callback( event, x, y, flags, params ):
        if event == cv2.EVENT_LBUTTONDOWN:
            print "(" + str(x) + " " + str(y) + ")"
            goal_pos[0] = x
            goal_pos[1] = y
    
    cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("img", click_callback, goal_pos )
    cont = True
    while cont:
        cv2.circle( img, (goal_pos[0], goal_pos[1]), 10, (0,255,0), 2 )
        cv2.imshow("img", img)
        if cv2.waitKey(300) & 0xFF == ord('q'):
            cont = False
    
    cv2.destroyWindow("img")
