#!/usr/bin/env python
from re import I
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
import yaml
from colorama import Fore, Back, Style

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

def my_callback(img, pub): 
    
  
    if  rospy.get_param("/use_rs_gazebo")=="true":

        # Read camera intrinsic calibration matrix
        with open('/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/visual_processing/config/realsense_gazebo_intrinsic.yaml', 'r') as file:
            camera_parameters = yaml.load(file)
            intrinsic_parameters = camera_parameters["camera_matrix"]["data"]
        K=np.array(intrinsic_parameters).reshape((3,3))
        print("Camera intrinsic parameters:"+ str(K))

    else:
        # Read camera intrinsic calibration matrix
        with open('/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/visual_processing/config/realsense_internal_intrinsic.yaml', 'r') as file:
            camera_parameters = yaml.load(file)
            intrinsic_parameters = camera_parameters["camera_matrix"]["data"]
        K=np.array(intrinsic_parameters).reshape((3,3))
        print("Camera intrinsic parameters:"+ str(K))

    #Read the  image size, encoding parameters and camera instrinsic parameters
    rospy.loginfo("New image, size = " + str(img.height) + " x " + str(img.width) + ", encoding = " + img.encoding)
    
    #Detect tags in OpenCV 
    cv_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11, tag25h9, tag16h5")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

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
        
        cv2.putText(cv_image, tagFamily, (ptA[0] - 30, ptA[1] - 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        cv2.putText(cv_image,  "ID:" + str(tagID), (ptA[0] - 30, ptA[1] - 5),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
        

        print ("[INFO] tag family: {}".format(tagFamily))
        print("[INFO] tag ID: {}".format(tagID))
        print(Fore.YELLOW +"Pixel coordinates:")
        print("A = (" + str(ptA[0]) + ", " + str(ptA[1]) + ")")
        print("B = (" + str(ptB[0]) + ", " + str(ptB[1]) + ")")
        print("C = (" + str(ptC[0]) + ", " + str(ptC[1]) + ")")
        print("D = (" + str(ptD[0]) + ", " + str(ptD[1]) + ")" + Style.RESET_ALL)

        # draw the bounding box of the AprilTag detection
        cv2.line(cv_image, ptA, ptB, (255, 255, 0), 2)
        cv2.line(cv_image, ptB, ptC, (255, 255, 0), 2)
        cv2.line(cv_image, ptC, ptD, (255, 255, 0), 2)
        cv2.line(cv_image, ptD, ptA, (255, 255, 0), 2)
        
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(cv_image, (cX, cY), 5, (0, 255, 255), -1)

        #Define coordinate system in the center of the tag model
        if (tagFamily=="tag36h11"):
            w=h=0.083 #m
        
        if (tagFamily=="tag25h9"):
            w=h=0.05 #m
        
        if (tagFamily=="tag16h5"):
            w=h=0.117 #m
        
        A_modelo=[w/2,-h/2,0] 
        B_modelo=[-w/2,-h/2,0] 
        C_modelo=[-w/2,h/2,0]
        D_modelo=[w/2,h/2,0]
        object_points=np.array([A_modelo,B_modelo,C_modelo,D_modelo], dtype=np.float64)
        image_points=np.array([ptA,ptB,ptC,ptD], dtype=np.float64)

        print(Fore.LIGHTCYAN_EX + "object_points=" + str(object_points) + Style.RESET_ALL)
        print(Fore.YELLOW + "image_points=" + str(image_points) + Style.RESET_ALL)

        #transform pixel coordinates to world
        success, rvec, tvec=cv2.solvePnP(object_points,image_points,K,0)
        print(Fore.LIGHTMAGENTA_EX + "rotation vector=" + str(rvec))
        print("translation vector=" + str(tvec) )
        
        # transform rotation vector 3x1 to  rotation matrix (3x3)
        rmat,_=cv2.Rodrigues(rvec)
        print("rotation matrix=" + str(rmat) + Style.RESET_ALL)

        #Convertir matriz de rotacion y vector de translacion en matriz homogenea
        t=np.c_[rmat,tvec]
        T=np.vstack([t,[0,0,0,1]])
        print(Fore.GREEN + "Transormation between camera and tag is:" + str(T))
        print("Matrix size is:"+ str(T.shape) + Style.RESET_ALL)

        #publish the tag frame in ROS
        tf=publish_transform(T,"camera_color_optical_frame" , "tag_frame" + str(i), img.header.stamp)
        i = i+1

        cv2.imshow("rgb camera",cv_image)

        cv2.waitKey(1)

        print("-------------------------------------------------------------------------------")

    #publish a OpenCV image to a Ros message
    imgmsg_image = CvBridge().cv2_to_imgmsg(cv_image, encoding="rgb8")
    pub.publish(imgmsg_image)

if __name__ == '__main__':
   rospy.init_node('my_node', anonymous=True)
   pub = rospy.Publisher('/my_apriltag_img', Image, queue_size=10)
   image_sub = rospy.Subscriber('/camera/color/image_raw', Image, my_callback, callback_args=pub)
   rospy.spin()
