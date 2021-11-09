#!/usr/bin/env python2
"""
@package tf_helper_functions
@author Hector Herrero
@brief Simple node for acting as tf listener and static transform broadcaster.

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import rospy
from tf_helper_functions.tf_listener_broadcaster import TFListenerBroadcaster

if __name__ == '__main__':
    tflb = TFListenerBroadcaster()
    rospy.loginfo('thf_tf_listener_broadcaster node started successfuly')
    rospy.spin()
