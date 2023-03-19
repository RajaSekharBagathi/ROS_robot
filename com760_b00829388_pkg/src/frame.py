#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math


class FixedTFBroadcaster:
     def __init__(self):
         self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
         
         while not rospy.is_shutdown():
               rospy.sleep(0.1)

               t = geometry_msgs.msg.TransformStamped()
               t.header.frame_id = "turtle1"
               t.header.stamp = rospy.Time.now()
               t.child_frame_id = "carrot1"
               t.transform.translation.x = -3.0 
               t.transform.translation.y = 1.0 
               t.transform.translation.z = 0.0
        
               t.transform.rotation.x = 0.0
               t.transform.rotation.y = 1.0
               t.transform.rotation.z = 0.0
               t.transform.rotation.w = 0.0

               p = geometry_msgs.msg.TransformStamped()
               p.header.frame_id = "turtle1"
               p.header.stamp = rospy.Time.now()
               p.child_frame_id = "carrot2"
               p.transform.translation.x = -3.0
               p.transform.translation.y = -1.0
               p.transform.translation.z = 0.0

               p.transform.rotation.x = 0.0
               p.transform.rotation.y = 1.0
               p.transform.rotation.z = 0.0
               p.transform.rotation.w = 0.0
               tfm = tf2_msgs.msg.TFMessage([t, p])
               self.pub_tf.publish(tfm)

               
if __name__ == '__main__':
   rospy.init_node('frame')
   tfb = FixedTFBroadcaster()

   rospy.spin()
