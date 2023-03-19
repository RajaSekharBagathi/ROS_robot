#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *
from com760_b00829388_pkg.srv import SetBugBehaviourStatus
import random
import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)
position_ = Point()
initial_position_ = Point()
initial_position_.x = random.randint(-4, 4)
initial_position_.y = random.randint(-4, 4)
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = 5
desired_position_.y = 4
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0

def clbk_odom(msg):
    global position_, yaw_
    position_ = msg.pose.pose.position
    quaternion = (msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {'right': min(min(msg.ranges[0:143]), 10),
               'fright': min(min(msg.ranges[144:287]), 10),
               'front': min(min(msg.ranges[288:431]), 10),
               'fleft': min(min(msg.ranges[432:575]), 10),
               'left': min(min(msg.ranges[576:719]), 10)}

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
       resp = srv_client_go_to_point_(True)
       resp = srv_client_wall_follower_(False)
    if state_== 1:
       resp = srv_client_go_to_point_(False)
       resp = srv_client_wall_follower_(True)
def normalize_angle(angle):
    if (math.fabs(angle) > math.pi):
       angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber('com760bot/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBugBehaviourStatus)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBugBehaviourStatus)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    model_state = ModelState()
    model_state.model_name = 'com760bot'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)

    change_state(0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
          if regions_ == None:
             continue
          if state_ == 0:
             if regions_['front'] > 0.15 and regions_['front'] < 1:
                change_state(1)
          elif state_ == 1:
               desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
               err_yaw = normalize_angle(desired_yaw - yaw_)

               if math.fabs(err_yaw) < (math.pi / 6) and \
                  regions_['front'] > 1.5 and regions_['fright'] > 1 and regions_['fleft'] > 1:
                  print 'less than 30'
                  change_state(0)
            
            # between 30 and 90
               if err_yaw > 0 and \
                  math.fabs(err_yaw) > (math.pi / 6) and \
                  math.fabs(err_yaw) < (math.pi / 2) and \
                  regions_['left'] > 1.5 and regions_['fleft'] > 1:
                  print 'between 30 and 90 - to the left'
                  change_state(0)
                
               if err_yaw < 0 and \
                  math.fabs(err_yaw) > (math.pi / 6) and \
                  math.fabs(err_yaw) < (math.pi / 2) and \
                  regions_['right'] > 1.5 and regions_['fright'] > 1:
                  print 'between 30 and 90 - to the right'
                  change_state(0)
  
          rate.sleep()

if __name__ == '__main__':
   main()
