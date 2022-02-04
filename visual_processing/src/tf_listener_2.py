#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
   
if __name__ == '__main__':
    rospy.init_node('tf_listener')
   
    listener = tf.TransformListener()
  
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
  
         try:
               (trans,rot) = listener.lookupTransform('/base_link', '/tag_frame', rospy.Time(0))
         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               continue
   
    print()
  
    rate.sleep()
