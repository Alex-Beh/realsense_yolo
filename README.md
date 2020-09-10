## Pixel convert to real distance:
Use **realsense-viewer** GUI to get the x,y & z real world distance and compare the value with the result from computation.

 1. From Intrinsic Camera Calibration
	 
	 **Idea:** From YOLO detection result get xmin,xmaxmymin & ymax:  [Link](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect)
> u = (xmin+xmax)/2; 
> v = (ymin+ymax)/2; 
> Z =0.001*(cv_ptr->image.at<u_int16_t>(v,u)); 
> Xreal = (u - cx) * Z / fx;  
> Yreal = (v - cy) * Z / fy; 
> Zreal = Z;
 2. PointCloud from PointCloud2 to get xyz
	 
	 **Idea:** Convert PointCloud2 to PointCloud and get the distance from the respective pixel: [Link](https://answers.ros.org/question/9239/reading-pointcloud2-in-c/)
> sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud_msg, out_cloud); 
> x_center = (i.m_bbox.xmin+i.m_bbox.xmax)/2; 
> y_center = (i.m_bbox.ymin+i.m_bbox.ymax)/2; 
> int  ind = (x_center) + (y_center)*pointcloud_msg->width; 
> Xreal = (float)out_cloud.points[ind].x; 
> Yreal = (float)out_cloud.points[ind].y; 
> Zreal = (float)out_cloud.points[ind].z;
-------------
<ins>Theora & Compressed Depth Image Transport **Error** while rosbag record</ins>
 - cv_bridge exception: '[16UC1] is not a color format. but [bgr8] is.
 - Compressed Depth Image Transport - Compression requires single-channel 32bit-floating point or 16bit raw depth images (input format is: rgb8).

Solution: [Link](https://github.com/IntelRealSense/realsense-ros/issues/315#issuecomment-531382378)

    rosbag record -a -O realsense_bag.bag -x "(.*)/compressed(.*)|.*(theora)"
