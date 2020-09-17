#pragma once

// c++
#include <vector>
#include <mutex>  
#include <math.h>       /* isnan, sqrt */

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

// darknet_ros
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// Opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Spencer
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>

// realsense_yolo_msgs
#include <realsense_yolo/debug_yolo.h>
#include <realsense_yolo/BoundingBoxes3d.h>

namespace realsense_yolo{
    
    typedef struct bbox_t_3d{
        bbox_t_3d(darknet_ros_msgs::BoundingBox bbox,float coord)
            :m_bbox(bbox),m_coord(coord){}

        darknet_ros_msgs::BoundingBox m_bbox;
        float m_coord;                      // depth distance 
    } bbox_t_3d;

    typedef struct camera_info_vector{
        float m_fx,m_fy,m_cx,m_cy;
    } camera_info_vector;

    class realsense_yolo_detector{
        public:
            // Constructor
            explicit realsense_yolo_detector(ros::NodeHandle& nodeHandle_,const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
            // Destructor
            ~realsense_yolo_detector();
        
        private:
            ros::NodeHandle nodeHandle_;

	        ros::Publisher people_position_pub,bbox3d_pub,debug_yolo_pub;
		    image_transport::Publisher yolo_image_pub;
            tf::TransformListener tfListener_;

            // Parameter for nh.param()
            std::string detection_output_pub, camera_link, marker_array_topic, depth_topic,pointcloud2_topic;
            float probability_threshold;
            std::vector<std::string> obj_list;

            double LARGE_VARIANCE_;
            double pose_variance_;
            int detection_id_increment_, detection_id_offset_, current_detection_id_; // added for multi-sensor use in SPENCER
            
            // Global variable
            std::mutex data_lock;
            camera_info_vector intrinsic_camera_matrix;         // ?? why don't need constructor in the typedef struct 
            realsense_yolo::debug_yolo debug_message;
            bool boudingbox_pcl = false;

            message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> m_yolo_detection_result_sub;
            message_filters::Subscriber<sensor_msgs::Image> m_depth_img_sub;
            message_filters::Subscriber<sensor_msgs::PointCloud2> m_pointcloud_sub;

            typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image,sensor_msgs::PointCloud2> SyncPolicy;
            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            boost::shared_ptr<Synchronizer> sync;
            
            // Function & Callback function
            void init();
            void filterOutUnwantedDetections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_detection_raw_result, const sensor_msgs::Image::ConstPtr& depth_image,
                    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
            void camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
            spencer_tracking_msgs::DetectedPersons fillPeopleMessage(std::vector<bbox_t_3d> result_vec, std::string obj_names, std::string camera_frame_id,
                    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);

            void draw_boxes(const realsense_yolo::BoundingBoxes3d boxes);
            void calculate_boxes_pcl(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg,  std::vector<darknet_ros_msgs::BoundingBox> original_bboxes_);
            void calculate_boxes(std::vector<bbox_t_3d> result_vec);
    };
}
