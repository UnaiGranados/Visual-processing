#!/usr/bin/env python
from numpy.core.records import array
import message_filters
import cv2
import rospy
import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

# import tf
# import tf2_ros.transform_broadcaster

def my_callback(img, pub):
    rospy.loginfo("New image, size = " + str(img.height) + " x " + str(img.width) + ", encoding = " + img.encoding)
    K=np.array(rospy.get_param("/camera_matrix/data")).reshape((3,3))
    print("Camera intrinsic parameters:"+str(K))
    cv_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    # camera_info_K = np.array(camera_info.K).reshape([3, 3])
    # camera_info_D = np.array(camera_info.D)
    # rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

    # loop over the AprilTag detection results
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

    #publish a OpenCV image to a Ros message
    imgmsg_image = CvBridge().cv2_to_imgmsg(cv_image, encoding="rgb8")
    pub.publish(imgmsg_image)

    # br=tf.TransformBroadcaster()
    # br.sendTransform()

def my_callback_2(img_depth, pub):
    rospy.loginfo("New image, size = " + str(img_depth.height) + " x " + str(img_depth.width) + ", encoding = " + img_depth.encoding)
    cv_image = CvBridge().imgmsg_to_cv2(img_depth, desired_encoding="rgb8")
    

    #publish a OpenCV image to a Ros message
    imgmsg_image = CvBridge().cv2_to_imgmsg(cv_image, encoding="rgb8")
    pub.publish(imgmsg_image)

    # br=tf.TransformBroadcaster()
    # br.sendTransform()

if __name__ == '__main__':
   rospy.init_node('my_node', anonymous=True)
   pub = rospy.Publisher('/my_apriltag_img', Image, queue_size=10)
   pub_depth=rospy.Publisher('/depth',Image, queue_size=10)
   image_rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, my_callback, callback_args=pub)
   image_depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, my_callback_2, callback_args=pub)
   #image_sub = message_filters.Subscriber("image", Image)
   #depth_sub = message_filters.Subscriber("depth_image", Image)
   #sync = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)
   #sync.registerCallback(mask_detect_callback)

   rospy.spin()
