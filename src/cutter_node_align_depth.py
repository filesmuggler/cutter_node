#!/usr/bin/env python
"""
Aligns depth image with extracted RGB area creating new depth image
"""
__author__ = 'Krzysztof Stezala <krzysztof.stezala at student.put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys,time
import rospy
import roslib
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

class CutterNode:
    def __init__(self):
        rospy.loginfo("Cutter node ALIGN_DEPTH -> is RUN")
        self.image = np.zeros([480,640])
        self.depth = np.zeros([480,640])
        self.depth_output = np.zeros([480,640])

        self.depth_timestamp = 0
        self.header = Header()

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.depth_aligned_pub = rospy.Publisher("/cutter_node_align_depth",Image, queue_size=10)
        # cv bridge
        self.cv_bridge = CvBridge()
        # subscribed Topic
        self.image_subscriber = rospy.Subscriber("/extracted_image",Image, self.callback_image,queue_size=1)
        self.depth_subscriber = rospy.Subscriber("/align_depth",Image,self.callback_depth,queue_size=1)
                

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
        thresh1_norm = cv2.normalize(thresh1,thresh1,0,1,cv2.NORM_MINMAX)
        self.depth_output =  thresh1_norm * self.depth
        
        try:
            self.align_message = self.cv_bridge.cv2_to_imgmsg(self.depth_output, "16UC1")
            self.align_message.header.stamp = self.depth_timestamp
            self.align_message.header.frame_id = "map"
            self.align_message.header = self.header
            self.depth_aligned_pub.publish(self.align_message)
        except CvBridgeError as e:
            print(e)

    def callback_depth(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "16UC1")
            self.depth_timestamp = data.header.stamp
            self.header = data.header
            self.depth = cv_image
        except CvBridgeError as e:
            print("cv bridge: ",e)
 

def main(args):
    rospy.init_node('cutter_node_align_depth',anonymous=True)
    cn = CutterNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS cutter node")

if __name__ == "__main__":
    main(sys.argv)
