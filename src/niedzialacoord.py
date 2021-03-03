#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import math

from tf.transformations import quaternion_from_euler
import geometry_msgs.msg


def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray
    Reshapes the returned array to have shape (height, width), even if the height is 1.
    The reason for using np.fromstring rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = ros_numpy.point_cloud2.fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)] == ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)]]

    # if squeeze and cloud_msg.height == 1:
    #     return np.reshape(cloud_arr, (cloud_msg.width,))
    # else:
    #     return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 

    return cloud_arr

def callback(data):
    pcl2_array = pointcloud2_to_array(data)
    
    pcl2_new_array = []
    for point in pcl2_array:
        pcl2_new_array.append(point)
        print(point)
    
    #print(pcl2_new_array)

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

            print("center: ",avg_obj_center)
            # print(math.sqrt((abs(avg_obj_center[0])+0.22)**2+avg_obj_center[1]**2))
            # if math.sqrt((abs(avg_obj_center[0])+0.22)**2+avg_obj_center[1]**2) > r:
            #     print("object to far")

            print(math.sqrt((avg_obj_center[0])**2+(avg_obj_center[1])**2+(avg_obj_center[2])**2))

        roll=0.0
        pitch=3.14
        yaw=1.54
        q = quaternion_from_euler(roll,pitch,yaw)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = avg_obj_center[0]
        pose.position.y = avg_obj_center[1]
        pose.position.z = avg_obj_center[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        
        pub = rospy.Publisher("/objectcoord",geometry_msgs.msg.Pose,queue_size=10)
        while not rospy.is_shutdown():
            pub.publish(pose)
            rospy.Rate(2).sleep()
        
    else:
        print("no object within reach of the camera")



def main():
    try:
        rospy.init_node('filter_pcl',anonymous=True)
        rospy.Subscriber("/cutter_node_point_cloud_2",PointCloud2,callback)
        rospy.spin()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()