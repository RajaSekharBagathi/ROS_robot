#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from std_srvs.srv import Empty


class PID_control:
      def __init__(self):
          rospy.init_node('move', anonymous=True)
          self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
          self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
          self.pose = Empty()
          self.rate = rospy.Rate(10)

      def update_pose(self, data):
          self.pose = data
          self.pose.x = round(self.pose.x, 4)
          self.pose.y = round(self.pose.y, 4)
          
      def euclidean_distance(self, goal_pose):
          return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

      def linear_vel(self, goal_pose, constant = 1.5):
          return constant * self.euclidean_distance(goal_pose)

      def streeing_angle(self, goal_pose):
          return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

      def angular_vel(self, goal_pose, constant=6):
          return constant * (self.streeing_angle(goal_pose) - self.pose.theta)
      
      def move2goal(self):
          goal_pose = Pose()
          goal_pose.x = float(input("set your x goal in float: "))
          goal_pose.y = float(input("set your y goal in float: "))

          distance_tolerance = input("set your tolerance (insert a number greater than 0 (e.g. 0.01)): ")
          vel_msg = Twist()
          
          while self.euclidean_distance(goal_pose) >= distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)
                self.velocity_publisher.publish(vel_msg)
                rospy.spin()
      
            

if __name__ == '__main__':
   try:
       x = PID_control()
       x.move2goal()
      
   except rospy.ROSInterruptException:
        pass
