#include <ros/ros.h>

// #include <vision_msgs/Detection2DArray.h>

#include <yolov4_trt_ros/Detector2DArray.h>

#include <realsense_yolo/BoundingBox3d.h>
#include <realsense_yolo/BoundingBoxes3d.h>

class MobileNetAdaptor{
    public:
        MobileNetAdaptor(ros::NodeHandle &nh);

    private:
        ros::NodeHandle node_handle_;
        ros::Publisher results_pub;
        ros::Subscriber dnn_results_sub ;

        void fillYoloMessage(const yolov4_trt_ros::Detector2DArray::ConstPtr& detection_result);
};

MobileNetAdaptor::MobileNetAdaptor(ros::NodeHandle &nh): node_handle_(nh){
    results_pub = nh.advertise<realsense_yolo::BoundingBoxes3d>("/darknet_ros/bounding_boxes", 20);
    dnn_results_sub = nh.subscribe("/detections", 1000, &MobileNetAdaptor::fillYoloMessage,this);
}

void MobileNetAdaptor::fillYoloMessage(const yolov4_trt_ros::Detector2DArray::ConstPtr& detection_result){
    realsense_yolo::BoundingBoxes3d yolo_boxes;
    yolo_boxes.header = detection_result->header;

    for(auto &it:detection_result->detections){
        realsense_yolo::BoundingBox3d yolo_box;
        yolo_box.Class = it.results.class_name;
        yolo_box.probability = it.results.score;
        yolo_box.xmin = it.bbox.center.x - it.bbox.size_x/2;
        yolo_box.ymin = it.bbox.center.y - it.bbox.size_y/2;
        yolo_box.xmax = it.bbox.center.x + it.bbox.size_x/2;
        yolo_box.ymax = it.bbox.center.y + it.bbox.size_y/2;
        yolo_boxes.bounding_boxes.emplace_back(yolo_box);
    }

    results_pub.publish(yolo_boxes);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "yolov4_tensorRT_adaptor");
    ros::NodeHandle nh("~");

    //MobileNet-SSD part
    MobileNetAdaptor MobileNetAdaptor(nh);

    ROS_INFO("Adaptor of TensorRT Yolov4 detector to YOLO message start!");

    ros::spin();
    return 0;
}
