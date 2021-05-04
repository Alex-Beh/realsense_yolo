#include <ros/ros.h>

#include <dnn_detect/DetectedObjectArray.h>

#include <realsense_yolo/BoundingBox3d.h>
#include <realsense_yolo/BoundingBoxes3d.h>

class MobilenetSSDAdaptor {
  public:
    MobilenetSSDAdaptor(ros::NodeHandle &nh);

  private:
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_results_pub;
    ros::Subscriber m_dectectionResults_sub ;

    void fillAdaptorMessage(const dnn_detect::DetectedObjectArray::ConstPtr &mobilessd_result);
};

MobilenetSSDAdaptor::MobilenetSSDAdaptor(ros::NodeHandle &nh) : m_nodeHandle(nh) {
  m_results_pub = nh.advertise<realsense_yolo::BoundingBoxes3d>("/darknet_ros/bounding_boxes", 20);
  m_dectectionResults_sub = nh.subscribe("/dnn_objects", 1000,&MobilenetSSDAdaptor::fillAdaptorMessage, this);
}

void MobilenetSSDAdaptor::fillAdaptorMessage(
    const dnn_detect::DetectedObjectArray::ConstPtr &mobilessd_result) {

      realsense_yolo::BoundingBoxes3d yolo_boxes;
      yolo_boxes.header = mobilessd_result->header;

      for (auto &it : mobilessd_result->objects) {
        realsense_yolo::BoundingBox3d yolo_box;

        yolo_box.Class = it.class_name;
        yolo_box.probability = it.confidence;
        yolo_box.xmin = it.x_min;
        yolo_box.ymin = it.y_min;
        yolo_box.xmax = it.x_max;
        yolo_box.ymax = it.y_max;
        yolo_boxes.bounding_boxes.emplace_back(yolo_box);
      }

      m_results_pub.publish(yolo_boxes);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mobilessdNet_adaptor");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10);

  // MobileNet-SSD part
  MobilenetSSDAdaptor MobilenetSSDAdaptor(nh);

  ROS_INFO("Adaptor of MobileNet SSD detector to YOLO message start!");

  ros::spin();

  return 0;
}
