#!/usr/bin/env python

import sys,time

import rospy
import roslib
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

import cv2

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

#np.set_printoptions(threshold=sys.maxsize)

class CutterNode:
    def __init__(self):
        self.image = np.zeros([480,640])
        self.depth = np.zeros([480,640])
        self.depth_output = np.zeros([480,640])
        self.camera_info_timestamp = 0
        self.camera_info_dump = CameraInfo()

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.depth_aligned_pub = rospy.Publisher("/cutter_node_align_depth",Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/cutter_node_camera_info",CameraInfo, queue_size=1)
        # cv bridge
        self.cv_bridge = CvBridge()
        # subscribed Topic
        self.image_subscriber = rospy.Subscriber("/grabcut",Image, self.callback_image, queue_size=1)
        self.depth_subscriber = rospy.Subscriber("/align_depth",Image, self.callback_depth, queue_size=1)
        self.camera_info_subscriber = rospy.Subscriber("/camera_info",CameraInfo, self.callback_info, queue_size=1)


    def callback_image(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "mono8")
            self.image = cv_image
        except CvBridgeError as e:
            print(e)
        
        # compare two images

        self.depth_output = np.zeros([480,640])

        ret,thresh1 = cv2.threshold(cv_image,10.0,255.0,cv2.THRESH_BINARY)

        start = time.time()
        
        self.depth_output =  thresh1 * self.depth * 1.0 / 255.0
        
        end = time.time()
        #print(end - start)
        try:
            self.align_message = self.cv_bridge.cv2_to_imgmsg(self.depth_output, "32FC1")
            self.align_message.header.frame_id = "map"
            #self.align_message.header.stamp = self.camera_info_dump.header.stamp
            self.depth_aligned_pub.publish(self.align_message)
            #self.camera_info_pub.publish(self.camera_info_dump)
            # before
            #self.depth_aligned_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.depth_output, "32FC1"))
        except CvBridgeError as e:
            print(e)
        ## debug
        #print(thresh1)
        cv2.imshow("cutter_node_depth", self.depth_output)
        cv2.imshow("cutter_node_grey", thresh1)
        cv2.waitKey(3)

    def callback_depth(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "32FC1")
            # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
            cv_image_array = np.array(cv_image, dtype = np.dtype('f4'))
            #cv_image_array = cv_image_array * 255.0
            # Normalize the depth image to fall between 0 (black) and 1 (white)            
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            self.depth = cv_image_norm
        except CvBridgeError as e:
            print(e)

    def callback_info(self,data):
        try:
            self.camera_info_dump = data
            #self.camera_info_timestamp = data.header.stamp
        except e:
            print(e)
        
def main(args):
    ## debug
    cv2.namedWindow('cutter_node_depth')
    cv2.namedWindow('cutter_node_grey')
    rospy.init_node('cutter_node',anonymous=True)
    cn = CutterNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS cutter node")
    ## debug
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)