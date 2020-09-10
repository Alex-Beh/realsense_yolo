#include <ros/ros.h>
#include <realsense_yolo/realsense_yolo.hpp>
#include <string>

int main(int argc, char** argv) {

    ros::init(argc, argv, "realsense_yolo_detector");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);
    
    std::string camera_info;
    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info_msg;
    nh.param("camera_info_name", camera_info, std::string("/spencer/sensors/rgbd_front_top/depth/camera_info"));

    camera_info_msg =ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info,nh);
    
    realsense_yolo::realsense_yolo_detector realsense_yolo_detector(nh,camera_info_msg);

    ROS_INFO("realsense yolo detector start");

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}