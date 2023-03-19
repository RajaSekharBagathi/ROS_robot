#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from com760_b00829388_pkg.srv import SetBugBehaviourStatus

import math

class GoToPoint:
    def __init__(self):
        self.active = False
        # robot state variables
        self.position = Point()
        self.yaw = 0
        
        self.state = 0
        # Destination
        self.desired_position = Point()
        self.desired_position.x = rospy.get_param('des_pos_x')
        self.desired_position.y = rospy.get_param('des_pos_y')
        self.desired_position.z = 0
        # Threshold parameters
        self.yaw_threshold = math.pi/90 # convert to radian		
        self.dist_threshold = rospy.get_param('th_dist') # unit: meter

        # publishers
        self.pub_vel = rospy.Publisher('/com760bot/cmd_vel', Twist, queue_size=1)    
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.srv = rospy.Service('go_to_point_switch', SetBugBehaviourStatus, self.go_to_point_switch)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                continue
            else:
                if self.state == 0:
                    self.fix_heading(self.desired_position)
                elif self.state == 1:
                    self.go_straight(self.desired_position)
                elif self.state == 2:
                    self.done()
                else:
                    rospy.logerr('Unknown state!')
        
            rate.sleep()

    # callbacks
    def callback_odom(self,msg):
        # reading position
        self.position = msg.pose.pose.position
    
        # yaw
        quaternion = (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    
    def change_state(self,state):
        if state is not self.state:
            print ('Wall follower - [%s] - %s' % (state, state))
            self.state = state

    def normalize_angle(self,angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_heading(self,des_pos):  
        print(self.position.x,self.position.y)      
        print(des_pos.x,des_pos.y)      
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)    

        print(err_yaw)
            
        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_threshold:
            twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
            self.pub_vel.publish(twist_msg)
    
        # state change conditions
        elif math.fabs(err_yaw) <= self.yaw_threshold:
            print ('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def go_straight(self,des_pos): 
        print(self.position.x,self.position.y)      
        print(des_pos.x,des_pos.y)
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = desired_yaw - self.yaw_threshold
        err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))
    
        if err_pos > self.dist_threshold:
            twist_msg = Twist()
            twist_msg.linear.x = 0.6
            twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
            self.pub_vel.publish(twist_msg)
        else:
            print ('Error in position: [%s]' % err_pos)
            self.change_state(2)
    
        # state change conditions
        if math.fabs(err_yaw) > self.yaw_threshold:
            print ('Error in Yaw: [%s]' % err_yaw)
            self.change_state(0)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        print ('Go to point executed successfully')
        self.pub_vel.publish(twist_msg)

    def go_to_point_switch(self,req):
        self.active = req.flag
        res = SetBugBehaviourStatusResponse()        
        res.message = 'Done!'
        return res

if __name__ == '__main__':

    rospy.init_node('go_to_point')
    GoToPoint()
    rospy.spin()
