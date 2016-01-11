##/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

class TurtlebotController:

    def __init__(self, kp = 1.0 ):
        rospy.init_node('turtlebot_controller')
        self.pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
        self.kp = kp

    def goto(self, goal_pos, current_pos, goal_orientation, current_orientation):
        control_speed = 0
        control_turn = 0

        twist = Twist()
        twist.linear.x = control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = control_turn
