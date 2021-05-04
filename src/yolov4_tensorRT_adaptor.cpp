#include <ros/ros.h>

#include <yolov4_trt_ros/Detector2DArray.h>

#include <realsense_yolo/BoundingBox3d.h>
#include <realsense_yolo/BoundingBoxes3d.h>

class Yolov4Adaptor{
    public:
        Yolov4Adaptor(ros::NodeHandle &nh);

    private:
        ros::NodeHandle m_nodeHandle;
        ros::Publisher m_results_pub;
        ros::Subscriber m_dectectionResults_sub ;

        void fillAdaptorMessage(const yolov4_trt_ros::Detector2DArray::ConstPtr& detection_result);
};

Yolov4Adaptor::Yolov4Adaptor(ros::NodeHandle &nh): m_nodeHandle(nh){
    m_results_pub = nh.advertise<realsense_yolo::BoundingBoxes3d>("/darknet_ros/bounding_boxes", 20);
    m_dectectionResults_sub = nh.subscribe("/detections", 1000, &Yolov4Adaptor::fillAdaptorMessage,this);
}

void Yolov4Adaptor::fillAdaptorMessage(const yolov4_trt_ros::Detector2DArray::ConstPtr& detection_result){
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

    m_results_pub.publish(yolo_boxes);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolov4_tensorRT_adaptor");
    ros::NodeHandle nh("~");

    //Yolo_v4 Adaptor part
    Yolov4Adaptor Yolov4Adaptor(nh);

    ROS_INFO("Adaptor of TensorRT Yolov4 detector to YOLO message start!");

    ros::spin();
    return 0;
}
