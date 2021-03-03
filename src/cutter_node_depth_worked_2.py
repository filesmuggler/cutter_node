#!/usr/bin/env python

import sys,time

import rospy
import roslib
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

import cv2

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

class CutterNode:
    def __init__(self):
        self.image = np.zeros([480,640])
        self.depth = np.zeros([480,640])
        self.depth_output = np.zeros([480,640])

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.depth_aligned_pub = rospy.Publisher("/cutter_node_align_depth",Image, queue_size=10)
        # cv bridge
        self.cv_bridge = CvBridge()
        # subscribed Topic
        self.image_subscriber = rospy.Subscriber("/grabcut",Image, self.callback_image, queue_size=1)
        self.depth_subscriber = rospy.Subscriber("/align_depth",Image, self.callback_depth, queue_size=1)
        

    def callback_image(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "mono8")
            self.image = cv_image
        except CvBridgeError as e:
            print(e)
        
        # compare two images

        self.depth_output = np.array(np.zeros([480,640]), dtype = np.dtype('f4'))

        ret,thresh1 = cv2.threshold(cv_image,10.0,255.0,cv2.THRESH_BINARY)
        
        depth_out =  thresh1 * self.depth 

        #depth_out = np.float32(depth_out)

        self.depth_output = cv2.normalize(depth_out, depth_out, 0, 1, cv2.NORM_MINMAX)

        self.depth_output = np.float32(self.depth_output)
        
        try:
            self.align_message = self.cv_bridge.cv2_to_imgmsg(self.depth_output, "32FC1")
            self.align_message.header.frame_id = "map"
            self.depth_aligned_pub.publish(self.align_message)
        except CvBridgeError as e:
            print(e)
        cv2.imshow("cutter_node_depth_output", self.depth_output)
        cv2.imshow("cutter_node_mask", thresh1)
        cv2.waitKey(3)

    def callback_depth(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "32FC1")
            # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
            cv_image_array = np.array(cv_image, dtype = np.dtype('f4'))
            # Normalize the depth image to fall between 0 (black) and 1 (white)            
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            cv_image_norm = np.float32(cv_image_norm)
            self.depth = cv_image_norm
        except CvBridgeError as e:
            print("cv bridge: ",e)

        
        

def main(args):
    ## debug
    cv2.namedWindow('cutter_node_depth_output')
    cv2.namedWindow('cutter_node_mask')
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