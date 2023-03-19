#!/usr/bin/env python

import rospy

import math

import tf2_ros

import geometry_msgs.msg

import turtlesim.srv

import random

if __name__ == '__main__':
   rospy.init_node('listener')
   tfBuffer = tf2_ros.Buffer()
   listener = tf2_ros.TransformListener(tfBuffer)

   rospy.wait_for_service('spawn')

   spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
   i = random.randint(1,9)
   j = random.randint(1,9)
   k = random.randint(1,9)
   spawner(i, j, k, 'turtle2')

   spawner1 = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
   spawner1(k, j, i, 'turtle3')

   turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
   turtle_vel1 =  rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
  
   rospy.wait_for_service("/turtle2/set_pen")
   setpen = rospy.ServiceProxy("/turtle2/set_pen", turtlesim.srv.SetPen)
   setpen(0, 0, 255, 3, 0)#blue
   rospy.wait_for_service("/turtle3/set_pen")
   setpen1 = rospy.ServiceProxy("/turtle3/set_pen", turtlesim.srv.SetPen)
   setpen1(0, 255, 255, 3, 0)

   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():

        try:
            trans = tfBuffer.lookup_transform('turtle2', 'carrot1', rospy.Time())
        
            trans1 = tfBuffer.lookup_transform('turtle3', 'carrot2', rospy.Time())  
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
           rate.sleep()
           continue
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        cmd = geometry_msgs.msg.Twist()
        cmd.angular.z = 4 * math.atan2(trans1.transform.translation.y, trans.transform.translation.x)
        cmd.linear.x = 0.5 * math.sqrt(trans1.transform.translation.x ** 2 + trans1.transform.translation.y ** 2)

        turtle_vel.publish(msg)
        turtle_vel1.publish(cmd)
        rate.sleep()
