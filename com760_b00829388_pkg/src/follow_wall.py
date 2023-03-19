#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from com760_b00829388_pkg.srv import SetBugBehaviourStatus

import math


class FollowWall:
    
    def __init__(self):

        self.active = False
        self.state = 0
        self.regions = {
                        'right': 0,
                        'fright': 0,
                        'front': 0,
                        'fleft': 0,
                        'left': 0,
        }
        self.state_dict = {
                            0: 'find the wall',
                            1: 'turn left',
                            2: 'follow the wall',
        }
        print('code Running')
        self.pub_vel = rospy.Publisher('/com760bot/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber('/com760bot/laser/scan', LaserScan, self.callback_laser)
        
        self.srv = rospy.Service('wall_follower_switch', SetBugBehaviourStatus, self.wall_follower_switch)
    
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue
        
            msg = Twist()

            if self.state == 0:
                msg = self.find_wall()
            elif self.state == 1:
                msg = self.turn_left()
            elif self.state == 2:
                msg = self.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
            
            self.pub_vel.publish(msg)            
            rate.sleep()

    def callback_laser(self,msg):    
        
        self.regions = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }

        self.take_action()
    
    def change_state(self,state):
        if state is not self.state:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict[state]))
            self.state = state

    def take_action(self):
        
        twist_msg = Twist()
        linear_x = 0
        angular_z = 0
    
        state_description = ''
    
        d = 1.5
    
        if self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
            self.find_wall()
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
            self.turn_left()
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
            self.follow_the_wall()
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
            self.find_wall()
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
            self.turn_left()
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
            self.turn_left()
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
            self.turn_left()
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
            self.find_wall()
        else:
            state_description = 'unknown case'
        
        rospy.loginfo(self.regions)
        rospy.loginfo(state_description)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.3
        return msg

    def follow_the_wall(self):         
        msg = Twist()
        msg.linear.x = 0.5
        return msg
    # service callbacks
    def wall_follower_switch(self,req):
        self.active = req.flag
        res = SetBugBehaviourStatusResponse()
        res.message = 'Done!'
        return res

if __name__=='__main__':
    
    rospy.init_node('follow_wall')
    FollowWall()
    rospy.spin()
