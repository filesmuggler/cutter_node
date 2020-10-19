#!/usr/bin/env python
"""
Gives coordinates of the cut object in meters wrt frame_id
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

from tf.transformations import quaternion_from_euler
import geometry_msgs.msg

def callback(data):
    pcl2_new_array = []
    for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
        pcl2_new_array.append(point)
    avg_obj_center = [0.0,0.0,0.0]
    if len(pcl2_new_array) != 0:
        for point in pcl2_new_array:
            avg_obj_center[0]=avg_obj_center[0]+point[0]
            avg_obj_center[1]=avg_obj_center[1]+point[1]
            avg_obj_center[2]=avg_obj_center[2]+point[2]

        if(len(pcl2_new_array) is not 0):
            avg_obj_center[0]=avg_obj_center[0]*1.0/len(pcl2_new_array)
            avg_obj_center[1]=avg_obj_center[1]*1.0/len(pcl2_new_array)
            avg_obj_center[2]=avg_obj_center[2]*1.0/len(pcl2_new_array)

            # print("center: ",avg_obj_center)
            # print(math.sqrt((avg_obj_center[0])**2+(avg_obj_center[1])**2+(avg_obj_center[2])**2))

        roll=0.0
        pitch=3.14
        yaw=1.54
        q = quaternion_from_euler(roll,pitch,yaw)
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = avg_obj_center[0]
        pose.pose.position.y = avg_obj_center[1]
        pose.pose.position.z = avg_obj_center[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
	pose.header.frame_id = "map"
	pose.header.seq = 1
	pose.header.stamp = rospy.Time.now()

        
        pub = rospy.Publisher("/cutter_node_coord",geometry_msgs.msg.PoseStamped,queue_size=10)
	    #print("center: ",avg_obj_center)
        #print(math.sqrt((avg_obj_center[0])**2+(avg_obj_center[1])**2+(avg_obj_center[2])**2))
        pub.publish(pose)
        rospy.Rate(10).sleep()
        
    else:
        print("no object within reach of the camera")

def main():
    try:
        rospy.loginfo("Cutter node COORD -> is RUN")
        rospy.init_node('cutter_node_coord',anonymous=True)
        rospy.Subscriber("/cutter_node_points",PointCloud2,callback)
        rospy.spin()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
