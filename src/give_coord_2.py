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
        #print(point)
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
        rospy.init_node('give_coord',anonymous=True)
        rospy.Subscriber("/cutter_node_point_cloud_2",PointCloud2,callback)
        rospy.spin()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()