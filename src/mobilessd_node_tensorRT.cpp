#include <ros/ros.h>
#include <realsense_yolo/realsense_yolo.hpp>

// #include <dnn_detect/DetectedObjectArray.h>
#include <vision_msgs/Detection2DArray.h>

// darknet_ros
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class MobileNetAdaptor{
    public:
        MobileNetAdaptor(ros::NodeHandle &nh);

    private:
        ros::NodeHandle node_handle_;
        ros::Publisher results_pub;
        ros::Subscriber dnn_results_sub ;

        void fillYoloMessage(const vision_msgs::Detection2DArray::ConstPtr& mobilessd_result);

};

MobileNetAdaptor::MobileNetAdaptor(ros::NodeHandle &nh): node_handle_(nh){
    results_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 20);
    dnn_results_sub = nh.subscribe("/detectnet/detections", 1000, &MobileNetAdaptor::fillYoloMessage,this);
}

void MobileNetAdaptor::fillYoloMessage(const vision_msgs::Detection2DArray::ConstPtr& mobilessd_result){
    darknet_ros_msgs::BoundingBoxes yolo_boxes;
    yolo_boxes.header = mobilessd_result->header;

    for(auto &it:mobilessd_result->detections){
        darknet_ros_msgs::BoundingBox yolo_box;
        yolo_box.Class = it.header.frame_id;
        yolo_box.probability = it.results[0].score;
        yolo_box.xmin = it.bbox.center.x - it.bbox.size_x/2;
        yolo_box.ymin = it.bbox.center.y - it.bbox.size_y/2;
        yolo_box.xmax = it.bbox.center.x + it.bbox.size_x/2;
        yolo_box.ymax = it.bbox.center.y + it.bbox.size_y/2;
        yolo_boxes.bounding_boxes.emplace_back(yolo_box);
    }

    results_pub.publish(yolo_boxes);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mobilessd_adaptor");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    // YOLO part
    // std::string camera_info;
    // boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info_msg;
    // nh.param("camera_info_name", camera_info, std::string("/spencer/sensors/rgbd_front_top/depth/camera_info"));
    // camera_info_msg =ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info,nh);
    // realsense_yolo::realsense_yolo_detector realsense_yolo_detector(nh,camera_info_msg);

    //MobileNet-SSD part
    MobileNetAdaptor MobileNetAdaptor(nh);

    ROS_INFO("Adaptor of MobileNet SSD detector to YOLO message start!");

    // while(ros::ok()){
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();
    return 0;

}
