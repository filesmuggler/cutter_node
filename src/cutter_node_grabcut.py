#!/usr/bin/env python
"""
Cuts the image from camera feed using Grabcut algorithm
"""
__author__ = 'Krzysztof Stezala <krzysztof.stezala at student.put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys, time

import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

## bounding box <x,y,w,h>
bbox = [0,0,0,0]

## even_click checks if the bbox should be taken as new bbox
even_click = False
grabcut = False

#bridge = CvBridge()

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/extracted_image",Image, queue_size=10)
        self.bridge = CvBridge()
        self.if_pub = False
        self.img_grabcut = np.zeros([480,640])
        # subscribed Topic from the follower node
        self.subscriber = rospy.Subscriber("/processed_image_bgr8",Image, self.callback, queue_size=1)

    def mouse_callback(self, event, x, y, flags, param):
        global mouseX, mouseY
        if event == cv2.EVENT_LBUTTONDBLCLK:
            mouseX, mouseY = x, y
            global even_click
            if even_click:
                bbox[2] = mouseX - bbox[0]
                bbox[3] = mouseY - bbox[1]
                even_click = not even_click
                global grabcut
                grabcut = True
                print("clicked 2!")
                print(bbox)
            else:
                bbox[0] = mouseX
                bbox[1] = mouseY
                even_click = not even_click
                print("clicked 1!")
   
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("original_image", cv_image)
            cv2.waitKey(3)
            ##grabcut part
            global grabcut
            if grabcut:
                mask = np.zeros(cv_image.shape[:2], np.uint8)
                bgdModel = np.zeros((1, 65), np.float64)
                fgdModel = np.zeros((1, 65), np.float64)
                bbox_t = tuple(bbox)
                cv2.grabCut(cv_image, mask, bbox_t, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
                mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
                self.img_grabcut = cv_image * mask2[:, :, np.newaxis]
                grabcut = False
                self.if_pub = True
            if self.if_pub:
                cv2.namedWindow('extracted_image')
                cv2.imshow("extracted_image", self.img_grabcut)
                cv2.waitKey(3)
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img_grabcut, "bgr8"))
                except CvBridgeError as e:
                    print(e)
        except CvBridgeError as e:
            print(e)
            #cv_image = np.zeros([480,640])
            
        

def main(args):
    '''Initializes and cleanup ros node'''
    time.sleep(3)
    ic = image_feature()
    rospy.init_node('cutter_node_grabcut', anonymous=True)
    cv2.namedWindow('original_image')    
    cv2.setMouseCallback('original_image', ic.mouse_callback)
    try:
        rospy.loginfo("Cutter node GRABCUT -> is RUN")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image extractor module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
