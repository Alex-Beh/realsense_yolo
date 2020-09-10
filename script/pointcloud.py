#!/usr/bin/env python2
# coding:utf-8
 
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
 
def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data,skip_nans=True)
    print type(gen)
    print("-----")
    # print(len(list(gen)))
    for p in gen:
        print p 
    print("-----")

 
def listener():
    rospy.init_node('pylistener', anonymous=True)
    rospy.Subscriber('/spencer/sensors/rgbd_front_top/depth_registered/points', PointCloud2, callback_pointcloud)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
