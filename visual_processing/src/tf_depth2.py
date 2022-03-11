#!/usr/bin/env python
# coding=utf-8
from numpy.core.records import array
import message_filters
import cv2
import tf2_ros
import pyquaternion as pyq
import rospy
import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import numpy as np
import imutils
import matplotlib.pyplot as plt  
import yaml
from colorama import Fore, Back, Style
import math
from visual_processing.srv import  ReturnNumberTags, ReturnNumberTagsResponse

num_tags = 0

class RealSense(object):
    def __init__(self):
      
        #use CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        # Node is subscribing to the camera/color/image_raw topic
        self.rgb_sub = message_filters.Subscriber('camera/color/image_raw', Image)

        # Node is subscribing to the /depth_to_rgb/image_raw topic
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

    def publish_transform(self, transf, base_link, child_link, time_stamp):
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
    
    num_tags = 0

    def tag_results_callback(self,req):
        print("tag_results service called")
        resp=ReturnNumberTagsResponse()
        resp.n_tags = num_tags
        return resp

    def callback(self, ros_rgb, ros_depth):

        
        if  rospy.get_param("use_rs_gazebo"):

            # Read  gazebo camera intrinsic calibration matrix
            with open('/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/visual_processing/config/realsense_gazebo_intrinsic.yaml', 'r') as file:
                camera_parameters = yaml.load(file)
                intrinsic_parameters = camera_parameters["camera_matrix"]["data"]
            K=np.array(intrinsic_parameters).reshape((3,3))
            print("Camera intrinsic parameters:"+ str(K))

        else:
            # Read real camera intrinsic calibration matrix
            with open('/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/visual_processing/config/realsense_internal_intrinsic.yaml', 'r') as file:
                camera_parameters = yaml.load(file)
                intrinsic_parameters = camera_parameters["camera_matrix"]["data"]
            K=np.array(intrinsic_parameters).reshape((3,3))
            print("Camera intrinsic parameters:"+ str(K))

        K=np.array(rospy.get_param("/camera_matrix/data")).reshape((3,3))
        f_x=K[0,0]
        f_y=K[1,1]
        c_x=K[0,2]
        c_y=K[1,2]

        rospy.loginfo("New rgb image, size = " + str(ros_rgb.width) + " x " + str(ros_rgb.height) + ", encoding = " + ros_rgb.encoding)
        rospy.loginfo("New depth image, size = " + str(ros_depth.width) + " x " + str(ros_depth.height) + ", encoding = " + ros_depth.encoding)

        #convert ROS image message to OpenCV image 
        depth_frame = self.br.imgmsg_to_cv2(ros_depth, desired_encoding="16UC1")
        rgb_frame = self.br.imgmsg_to_cv2(ros_rgb, desired_encoding="rgb8")
        
        # #resize image
        depth_frame = imutils.resize(depth_frame, width=640, height=480)
        rgb_frame = imutils.resize(rgb_frame, width=640, height=480)
        
        depth_frame_gray = cv2.cvtColor(depth_frame, cv2.COLOR_GRAY2BGR)
        rgb_gray =cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        
        options = apriltag.DetectorOptions(families="tag36h11, tag25h9, tag16h5")
        detector = apriltag.Detector(options)
        results = detector.detect(rgb_gray)
        
        global num_tags
        num_tags = len(results)
        print("Num_tags: " + str(num_tags))

        # loop over the AprilTag detection results
        T = np.eye(4)
        i = 1
        for r in results:

            tagID = r.tag_id
            if tagID != 0:
                continue

            # extract the bounding box (x, y)-coordinates for the AprilTag and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")

            cv2.putText(rgb_frame, tagFamily, (ptA[0] - 30, ptA[1] - 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.putText(rgb_frame,  "ID:" + str(tagID), (ptA[0] - 30, ptA[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)

            # draw the bounding box of the AprilTag detection
            cv2.line(rgb_frame, ptA, ptB, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptB, ptC, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptC, ptD, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptD, ptA, (255, 255, 0), 2)
        
            
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(rgb_frame, (cX, cY), 5, (0, 255, 255), -1)

            print("[INFO] tag family: {}".format(tagFamily))
            print(Fore.YELLOW + "A = (" + str(ptA[0]) + ", " + str(ptA[1])+ ")")
            T_tag_cam = np.eye(4)
            
            if ptA[0] >= 0 and ptA[0] < ros_depth.height and ptA[1] >= 0 and ptA[1] < ros_depth.width and ptB[0] >= 0 and ptB[0] < ros_depth.height and ptB[1] >= 0 and ptB[1] < ros_depth.width and ptC[0] >= 0 and ptC[0] < ros_depth.height and ptC[1] >= 0 and ptC[1] < ros_depth.width and ptD[0] >= 0 and ptD[0] < ros_depth.height and ptD[1] >= 0 and ptD[1] < ros_depth.width:

                if depth_frame[ptA[1], ptA[0]] != 0 and depth_frame[ptB[1], ptB[0]] != 0 and depth_frame[ptC[1], ptC[0]] != 0 and depth_frame[ptD[1], ptD[0]] != 0:
                   
                    print(" --> depth = " + str(depth_frame[ptA[1], ptA[0]]))
                    X_A_Cam= (depth_frame[ptA[1], ptA[0]] * ((ptA[0])-c_x))/f_x
                    Y_A_Cam= (depth_frame[ptA[1], ptA[0]] * ((ptA[1])-c_y))/f_y
                    ptA_Cam=(X_A_Cam,Y_A_Cam, depth_frame[ptA[1], ptA[0]])
                    print("Punto A respecto a la camara:" + str(ptA_Cam))

                    print("B = (" + str(ptB[0]) + ", " + str(ptB[1]) + ")")
                    print(" --> depth = " + str(depth_frame[ptB[1], ptB[0]]))
                    X_B_Cam= (depth_frame[ptB[1], ptB[0]] * ((ptB[0])-c_x))/f_x
                    Y_B_Cam= (depth_frame[ptB[1], ptB[0]] * ((ptB[1])-c_y))/f_y
                    ptB_Cam=(X_B_Cam,Y_B_Cam, depth_frame[ptB[1], ptB[0]])
                    print("Punto B respecto a la camara:" + str(ptB_Cam))

                    print("C = (" + str(ptC[0]) + ", " + str(ptC[1]) + ")")
                    print(" --> depth = " + str(depth_frame[ptC[1], ptC[0]]))
                    X_C_Cam=(depth_frame[ptC[1], ptC[0]] * ((ptC[0])-c_x))/f_x
                    Y_C_Cam=(depth_frame[ptC[1], ptC[0]] * ((ptC[1])-c_y))/f_y
                    ptC_Cam=(X_C_Cam,Y_C_Cam, depth_frame[ptC[1], ptC[0]])
                    print("Punto C respecto a la camara:" + str(ptC_Cam))

                    print("D = (" + str(ptD[0]) + ", " + str(ptD[1]) + ")")
                    print(" --> depth = " + str(depth_frame[ptD[1], ptD[0]]) )
                    X_D_Cam= (depth_frame[ptD[1], ptD[0]] * ((ptD[0])-c_x))/f_x
                    Y_D_Cam= (depth_frame[ptD[1], ptD[0]] * ((ptD[1])-c_y))/f_y
                    ptD_Cam= (X_D_Cam,Y_D_Cam, depth_frame[ptD[1], ptD[0]])
                    print("Punto D respecto a la camara:" + str(ptD_Cam))

                    print(Style.RESET_ALL)

                    PtA_Camara=np.array(ptA_Cam)
                    ptB_Camara=np.array(ptB_Cam)
                    ptC_Camara=np.array(ptC_Cam)
                    ptD_Camara=np.array(ptD_Cam)

                    ##Translation
                    P_cam=(PtA_Camara + ptB_Camara + ptC_Camara + ptD_Camara)/(4*1000) #mm to meter
                    print (Fore.LIGHTCYAN_EX + "Translation of tag from camera:" + str(P_cam) )

                    ##Rotation
                    X_tag_cam=-(ptB_Camara- PtA_Camara)/np.linalg.norm(ptB_Camara - PtA_Camara)
                    Y_tag_cam=-(ptD_Camara-PtA_Camara)/np.linalg.norm(ptD_Camara - PtA_Camara)
                    Z_tag_cam=np.cross(Y_tag_cam,X_tag_cam)/np.linalg.norm(np.cross(X_tag_cam,Y_tag_cam))
                    Y_tag_cam=np.cross(Z_tag_cam, X_tag_cam)
                    Mat=np.c_[X_tag_cam,Y_tag_cam,Z_tag_cam,P_cam]
                    print("Rotation matrix:" + str(Mat)+ Style.RESET_ALL)
                    
                    #Transformation
                    T_tag_cam=np.vstack([Mat,[0,0,0,1]])
                    print(Fore.GREEN + "Transormation between camera and tag is:" + str(T_tag_cam))
                    print("Matrix size is:"+ str(T_tag_cam.shape) + Style.RESET_ALL)
                    print("T * T^transp:"+ str(np.dot(T_tag_cam[:3, :3], T_tag_cam[:3, :3].T)) + Style.RESET_ALL)
                    print("norm x = " + str(np.linalg.norm(X_tag_cam)))
                    print("norm y = " + str(np.linalg.norm(Y_tag_cam)))
                    print("norm z = " + str(np.linalg.norm(Z_tag_cam)))


            #publish tag frame in ROS
            tf=self.publish_transform(T_tag_cam,"camera_color_optical_frame" , "tag_frame" + str(i), ros_rgb.header.stamp)
            i = i+1


            print("-------------------------------------------------------------------------------")

        #publish a OpenCV image to a Ros message
        imgmsg_image = CvBridge().cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        pub.publish(imgmsg_image)
                
def main():

    my_node = RealSense()
    rospy.init_node("tf_transform_rgb_depth", anonymous=True)
    ts = message_filters.ApproximateTimeSynchronizer([my_node.rgb_sub, my_node.depth_sub], 10, 0.1)
    ts.registerCallback(my_node.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    pub = rospy.Publisher('/my_apriltag_depth_img', Image, queue_size=10)
    main()