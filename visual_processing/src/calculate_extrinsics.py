#!/usr/bin/env python

from ast import Return
import sys
import copy
from tokenize import Double
import rospy

import moveit_msgs.msg
import geometry_msgs.msg
from  geometry_msgs.msg import Pose
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf2_ros import TransformListener
from tf2_ros import Buffer
import tf
from rospy import Time
from colorama import Fore, Back, Style
import pyquaternion as pyq
from geometry_msgs.msg import TransformStamped
from visual_processing.srv import  ReturnNumberTags, ReturnNumberTagsResponse
import math

def publish_transform(transf, base_link, child_link, time_stamp):
  br = tf2_ros.TransformBroadcaster()
  tf_msg = TransformStamped()
  tf_msg.header.stamp = time_stamp
  tf_msg.header.frame_id = base_link
  tf_msg.child_frame_id = child_link
  tf_msg.transform.translation.x = transf[0, 3]
  tf_msg.transform.translation.y = transf[1, 3]
  tf_msg.transform.translation.z = transf[2, 3]
  q = pyq.Quaternion(matrix=transf[0:3, 0:3])
  tf_msg.transform.rotation.w = q[0]
  tf_msg.transform.rotation.x = q[1]
  tf_msg.transform.rotation.y = q[2]
  tf_msg.transform.rotation.z = q[3]
  br.sendTransform(tf_msg)

def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def rotationMatrixToEulerAngles(R) :
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

#Transformation between camera_link and optical_frame
translation= np.array([0, 0.015, 0, 1])
e1=np.array([-1.571, -0.000, -1.571])
mat=np.eye(4)
mat[:3, :3]=eulerAnglesToRotationMatrix(e1)
mat[:,3]=translation.T
print(mat)
print(np.linalg.inv(mat))

#Transformation between  optical_frame and tool
translation2= np.array([0.023, -0.131, 0.103, 1])
e2=np.array([-0.055, 0, 0.430])
mat2=np.eye(4)
mat2[:3, :3]=eulerAnglesToRotationMatrix(e2)
mat2[:,3]=translation2.T
print(mat2)

#Transformation between camera_link and tool
T_CL_tool0 = np.dot(mat2, np.linalg.inv(mat))
print("T:"+ str(T_CL_tool0))
e1 = rotationMatrixToEulerAngles(T_CL_tool0[:3, :3])
print("Translation is :" + str(T_CL_tool0[:,3]) + "and rpy is:" + str(e1))