#!/usr/bin/env python
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


def callback_rgb(img, pub):
    #Read the  image size, encoding parameters and camera instrinsic parameters
    rospy.loginfo("New rgb image, size = " + str(img.height) + " x " + str(img.width) + ", encoding = " + img.encoding)
    K=np.array(rospy.get_param("/camera_matrix/data")).reshape((3,3))
    print("Camera intrinsic parameters:"+str(K))

    #Detect tags in OpenCV 
    cv_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    print (len(gray.shape))
    options = apriltag.DetectorOptions(families="tag36h11, tag25h9, tag16h5")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    # loop over the AprilTag detection results
    T = np.eye(4)
    i = 1
    for r in results:

        # extract the bounding box (x, y)-coordinates for the AprilTag and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(cv_image, tagFamily, (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        print("[INFO] tag family: {}".format(tagFamily))
        print("A = (" + str(ptA[0]) + ", " + str(ptA[1]) + ")")
        print("B = (" + str(ptB[0]) + ", " + str(ptB[1]) + ")")
        print("C = (" + str(ptC[0]) + ", " + str(ptC[1]) + ")")
        print("D = (" + str(ptD[0]) + ", " + str(ptD[1]) + ")")

        # draw the bounding box of the AprilTag detection
        cv2.line(cv_image, ptA, ptB, (255, 255, 0), 2)
        cv2.line(cv_image, ptB, ptC, (255, 255, 0), 2)
        cv2.line(cv_image, ptC, ptD, (255, 255, 0), 2)
        cv2.line(cv_image, ptD, ptA, (255, 255, 0), 2)
        
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(cv_image, (cX, cY), 5, (0, 255, 255), -1)

        #publish the tag frame in ROS
        tf=publish_transform(T,"camera_color_optical_frame" , "tag_frame" + str(i), img.header.stamp)
        i = i+1

    #publish a OpenCV image to a Ros message
    imgmsg_image = CvBridge().cv2_to_imgmsg(cv_image, encoding="rgb8")
    pub.publish(imgmsg_image)

    #publish the tag frame in ROS
    # tf=publish_transform(T,"camera_color_optical_frame" , "tag_frame", img.header.stamp)


def callback_depth(img_depth, pub):
    rospy.loginfo("New depth image, size = " + str(img_depth.height) + " x " + str(img_depth.width) + ", encoding = " + img_depth.encoding)
    cv_image_depth = CvBridge().imgmsg_to_cv2(img_depth, desired_encoding="passthrough")
    depth_image=cv_image_depth.astype(np.uint8)
    cv2.imshow("Depth window", depth_image)
    depth_array=np.array(cv_image_depth, dtype=np.float32)
    
    # rospy.loginfo(depth_array)
    print ("Depth values:" + str(depth_array))

    # publish a OpenCV image to a Ros message
    imgmsg_image = CvBridge().cv2_to_imgmsg(cv_image_depth, encoding="passthrough")
    pub.publish(imgmsg_image)


if __name__ == '__main__':
   rospy.init_node('my_node', anonymous=True)
   pub = rospy.Publisher('/my_apriltag_img', Image, queue_size=10)
   pub_depth=rospy.Publisher('/depth',Image, queue_size=10)
   image_rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback_rgb, callback_args=pub)
   image_depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback_depth, callback_args=pub)
   
   #image_sub = message_filters.Subscriber("image", Image)
   #depth_sub = message_filters.Subscriber("depth_image", Image)
   #sync = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)
   #sync.registerCallback(mask_detect_callback)

   rospy.spin()
