#!/usr/bin/env python

import sys,time

import rospy
import roslib
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

import cv2

import message_filters

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class DirtySync:
    def __init__(self):
        self.cam_publisher = rospy.Publisher("/camera_info_sync",CameraInfo,queue_size=10)
        self.depth_publisher = rospy.Publisher("/align_depth_sync",Image,queue_size=10)

        self.depth_subscriber = message_filters.Subscriber("/cutter_node_align_depth",Image)
        self.camera_info_subscriber = message_filters.Subscriber("/camera_info",CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_info_subscriber, self.depth_subscriber], 2, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.cameras_callback)
        self.rate = rospy.Rate(30)

    def cameras_callback(self, camera_msg, depth_msg):
        timestamp_cam = camera_msg.header.stamp
        timestamp_depth = depth_msg.header.stamp
        print("C: ", timestamp_cam, " D: ", timestamp_depth, " difference: ", timestamp_cam - timestamp_depth)
        depth_msg.header.stamp = rospy.Time()
        camera_msg.header.stamp = rospy.Time()

        self.cam_publisher.publish(camera_msg)
        self.depth_publisher.publish(depth_msg)
        
        

def main(args):
    rospy.init_node('dirty_sync',anonymous=True)
    ds = DirtySync()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS cutter node")

if __name__ == "__main__":
    main(sys.argv)