Command:
rosbag record -a -O realsense_bag.bag -x "(.*)/compressed(.*)|.*(theora)"

Theora & Compressed Depth Image Transport Error while rosbag record
cv_bridge exception: '[16UC1] is not a color format. but [bgr8] is. 
Compressed Depth Image Transport - Compression requires single-channel 32bit-floating point or 16bit raw depth images (input format is: rgb8).
https://github.com/IntelRealSense/realsense-ros/issues/315#issuecomment-531382378


sensor_msgs/PointCloud2
/spencer/sensors/rgbd_front_top/depth_registered/points
