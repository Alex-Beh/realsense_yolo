#include <ros/ros.h>

#include <vision_msgs/Detection2DArray.h>

#include <realsense_yolo/BoundingBox3d.h>
#include <realsense_yolo/BoundingBoxes3d.h>

class MobileNetAdaptor{
    public:
        MobileNetAdaptor(ros::NodeHandle &nh);

    private:
        ros::NodeHandle m_nodeHandle;
        ros::Publisher m_results_pub;
        ros::Subscriber m_dectectionResults_sub ;

        void fillAdaptorMessage(const vision_msgs::Detection2DArray::ConstPtr& mobilessd_result);
};

MobileNetAdaptor::MobileNetAdaptor(ros::NodeHandle &nh): m_nodeHandle(nh){
    m_results_pub = nh.advertise<realsense_yolo::BoundingBoxes3d>("/darknet_ros/bounding_boxes", 20);
    m_dectectionResults_sub = nh.subscribe("/detectnet/detections", 1000, &MobileNetAdaptor::fillAdaptorMessage,this);
}

void MobileNetAdaptor::fillAdaptorMessage(const vision_msgs::Detection2DArray::ConstPtr& mobilessd_result){
    realsense_yolo::BoundingBoxes3d yolo_boxes;
    yolo_boxes.header = mobilessd_result->header;

    for(auto &it:mobilessd_result->detections){
        realsense_yolo::BoundingBox3d yolo_box;
        yolo_box.Class = it.header.frame_id;
        yolo_box.probability = it.results[0].score;
        yolo_box.xmin = it.bbox.center.x - it.bbox.size_x/2;
        yolo_box.ymin = it.bbox.center.y - it.bbox.size_y/2;
        yolo_box.xmax = it.bbox.center.x + it.bbox.size_x/2;
        yolo_box.ymax = it.bbox.center.y + it.bbox.size_y/2;
        yolo_boxes.bounding_boxes.emplace_back(yolo_box);
    }

    m_results_pub.publish(yolo_boxes);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobilenetSSD tensorRT_adaptor");
    ros::NodeHandle nh("~");

    //MobileNet-SSD part
    MobileNetAdaptor MobileNetAdaptor(nh);

    ROS_INFO("Adaptor of TensorRT Mobilenet SSD detector to YOLO message start!");

    ros::spin();
    return 0;
}
