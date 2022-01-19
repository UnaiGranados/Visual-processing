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
import imutils
import matplotlib.pyplot as plt  

class RealSense(object):
    def __init__(self):
      
        #use CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Node is subscribing to the rgb/image_raw topic
        self.rgb_sub = message_filters.Subscriber('camera/color/image_raw', Image)

        # Node is subscribing to the /depth_to_rgb/image_raw topic
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)


    def callback(self, ros_rgb, ros_depth):

        rospy.loginfo("New rgb image, size = " + str(ros_rgb.width) + " x " + str(ros_rgb.height) + ", encoding = " + ros_rgb.encoding)
        rospy.loginfo("New depth image, size = " + str(ros_depth.width) + " x " + str(ros_depth.height) + ", encoding = " + ros_depth.encoding)

        #convert ROS image message to OpenCV image 
        depth_frame = self.br.imgmsg_to_cv2(ros_depth, desired_encoding="16UC1")
        rgb_frame = self.br.imgmsg_to_cv2(ros_rgb, desired_encoding="rgb8")
        
        # #resize image
        depth_frame = imutils.resize(depth_frame,height=480,  width=640)
        rgb_frame = imutils.resize(rgb_frame, height=480, width=640)
        
        depth_frame_gray = cv2.cvtColor(depth_frame, cv2.COLOR_GRAY2BGR)
        rgb_gray =cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        
        options = apriltag.DetectorOptions(families="tag36h11, tag25h9, tag16h5")
        detector = apriltag.Detector(options)
        results = detector.detect(rgb_gray)

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
            cv2.putText(rgb_frame, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            print("[INFO] tag family: {}".format(tagFamily))
            print("A = (" + str(ptA[0]) + ", " + str(ptA[1]))
            if ptA[0] >= 0 and ptA[0] < ros_depth.height and ptA[1] >= 0 and ptA[1] < ros_depth.width:
                print(" --> depth = " + str(depth_frame[ptA[0], ptA[1]]))
            print("B = (" + str(ptB[0]) + ", " + str(ptB[1]) + ")")
            print("C = (" + str(ptC[0]) + ", " + str(ptC[1]) + ")")
            print("D = (" + str(ptD[0]) + ", " + str(ptD[1]) + ")")

            # draw the bounding box of the AprilTag detection
            cv2.line(rgb_frame, ptA, ptB, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptB, ptC, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptC, ptD, (255, 255, 0), 2)
            cv2.line(rgb_frame, ptD, ptA, (255, 255, 0), 2)
        
            
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(rgb_frame, (cX, cY), 5, (0, 255, 255), -1)


            #publish the tag frame in ROS
            # tf=publish_transform(T,"camera_color_optical_frame" , "tag_frame" + str(i), ros_rgb.header.stamp)
            i = i+1

            #If we want to stablize, don't use imshow and waitKey(1). Use a publisher to send the data out and use rqt_gui to see the frame.
            cv2.imshow("depth camera", depth_frame)
            cv2.imshow("rgb camera", rgb_frame)
            
            cv2.waitKey(1)

        #publish a OpenCV image to a Ros message
        imgmsg_image = CvBridge().cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        pub.publish(imgmsg_image)
                
def main():

    my_node = RealSense()
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node("tf_transform_rgb_depth", anonymous=True)


    ts = message_filters.ApproximateTimeSynchronizer([my_node.rgb_sub, my_node.depth_sub], 10, 0.1)
    ts.registerCallback(my_node.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    pub = rospy.Publisher('/my_apriltag_depth_img', Image, queue_size=10)
    main()