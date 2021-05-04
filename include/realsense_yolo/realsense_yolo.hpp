#pragma once

// c++
#include <math.h> /* isnan, sqrt */
#include <vector>

// Opencv
#include <cv_bridge/cv_bridge.h>

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#ifdef WITH_SPENCER
// Spencer
#include <spencer_tracking_msgs/DetectedPerson.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#endif

// realsense_yolo_msgs
#include <realsense_yolo/BoundingBox3d.h>
#include <realsense_yolo/BoundingBoxes3d.h>

namespace realsense_yolo {

  typedef struct bbox_t_3d {
    bbox_t_3d(realsense_yolo::BoundingBox3d bbox, float coord)
        : m_bbox(bbox), m_coord(coord) {}

    realsense_yolo::BoundingBox3d m_bbox;
    float m_coord; // depth distance
  } bbox_t_3d;

  typedef struct camera_info_vector {
    float m_fx, m_fy, m_cx, m_cy;
  } camera_info_vector;

  class realsense_yolo_detector {
    public:
      explicit realsense_yolo_detector(ros::NodeHandle &nodeHandle_,const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg);
      ~realsense_yolo_detector();

    private:
      ros::NodeHandle nodeHandle_;

      ros::Publisher people_position_pub, bbox3d_pub;

      std::string detection_output_pub, camera_link, marker_array_topic,depth_topic;
      float probability_threshold;
      std::vector<std::string> obj_list;

      double LARGE_VARIANCE_;
      double pose_variance_;
      double currentTime;
      
      int detection_id_increment_, detection_id_offset_,current_detection_id_; 

      camera_info_vector intrinsic_camera_matrix;
      bool boudingbox_pcl = false;

      message_filters::Subscriber<realsense_yolo::BoundingBoxes3d> m_yolo_detection_result_sub;
      message_filters::Subscriber<sensor_msgs::Image> m_depth_img_sub;

      typedef message_filters::sync_policies::ApproximateTime<realsense_yolo::BoundingBoxes3d, sensor_msgs::Image> SyncPolicy;
      typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
      boost::shared_ptr<Synchronizer> sync;

      // Function & Callback function
      void init();

      void filterOutUnwantedDetections(const realsense_yolo::BoundingBoxes3d::ConstPtr &yolo_detection_raw_result,
                                  const sensor_msgs::Image::ConstPtr &depth_image);

      void camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg);

      #ifdef WITH_SPENCER
        spencer_tracking_msgs::DetectedPersons fillPeopleMessage(std::vector<bbox_t_3d> result_vec, std::string obj_names,
                          std::string camera_frame_id);
      #endif

      void draw_boxes(const realsense_yolo::BoundingBoxes3d boxes);
      void calculate_boxes(std::vector<bbox_t_3d> result_vec);
  };
} // namespace realsense_yolo
