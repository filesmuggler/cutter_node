#!/usr/bin/env python
"""
Gives tf of the cut object wrt frame_id
"""
__author__ = 'Krzysztof Stezala <krzysztof.stezala at student.put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import ros_numpy
import numpy as np
import math
import tf

from tf.transformations import quaternion_from_euler
import geometry_msgs.msg

import tf2_ros
import tf2_geometry_msgs 

def callback(data):
    # Publisher definition
    rospy.loginfo("Cutter node TF -> is RUN")
    pub_pose = rospy.Publisher("/cutter_node_tf", geometry_msgs.msg.PoseStamped, queue_size = 100)
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose = data

    tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("map",
		                           obj_pose.header.frame_id, #source frame
		                           rospy.Time(0), #get the tf at first available time
		                           rospy.Duration(2.0))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(obj_pose, transform)
    
    while not rospy.is_shutdown():    
	print("pose_transformed",pose_transformed.pose)
	br = tf.TransformBroadcaster()
	br.sendTransform((pose_transformed.pose.position.x,
			pose_transformed.pose.position.y,
			pose_transformed.pose.position.z), 
			(0,0,0,1),rospy.Time.now(),"object","map")

	pub_pose.publish(pose_transformed)
	rospy.Rate(30).sleep()     

def main():
    try:
        rospy.init_node('cutter_node_tf', anonymous = True)    
        sub_pose = rospy.Subscriber("/cutter_node_coord",geometry_msgs.msg.PoseStamped,callback)
        rospy.spin()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

    

if __name__ == '__main__':
    main()
