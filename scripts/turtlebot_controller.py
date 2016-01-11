##/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist

def get_orientation( pos_from, pos_to ):
    return np.arctan2( pos_to[1]-pos_from[1], pos_to[0]-pos_from[0] )
 
def get_distance( pos_from, pos_to ):
    return np.sqrt( (pos_from[0]-pos_to[0])**2 + (pos_from[1]-pos_to[1])**2 )

class TurtlebotController:

    def __init__(self, kp = [0.001, 0.01] ):
        #rospy.init_node('turtlebot_controller')
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.kp = kp
        self.step_distance = 5
  
        self.last_pos = None
        self.current_pos = None 

    def goto(self, goal_pos, pos):
        control_speed = 0
        control_turn = 0

        updated = self.update_pos( pos )
        goal_orientation = get_orientation( goal_pos, self.current_pos )
        current_orientation = self.estimate_orientation()

        
        if goal_pos[0] > 0 and goal_pos[1] >  0:
            print "GOAL [" + str(goal_pos[0]) + " " + str(goal_pos[1]) + "] CURRENT [" + str(self.current_pos[0]) + " " + str(self.current_pos[1]) + "]"
            print "GOAL " + str(goal_orientation) + " CURRENT " + str(current_orientation)
            delta_distance = get_distance( goal_pos, self.current_pos )
            delta_orientation = 0.0
            if goal_orientation != None and current_orientation != None:
                delta_orientation = goal_orientation - current_orientation 
 
            control_speed = 0.1 #self.kp[0] * delta_distance
            control_turn = self.kp[1] * delta_orientation        


            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn
            print "Linear=" + str(twist.linear.x) + " Angular=" + str(twist.angular.z) 
            self.pub.publish(twist)

        if updated == True:
            self.last_pos = self.current_pos

    def update_pos(self, pos):
        if self.current_pos == None:
            self.current_pos = pos
            return True
        else:
            if get_distance(self.current_pos, pos) > self.step_distance:
                self.current_pos = pos
                return True
        return False

    def estimate_orientation(self):
        if self.last_pos == None:
            return None
        else:
            return get_orientation( self.current_pos, self.last_pos )

  
