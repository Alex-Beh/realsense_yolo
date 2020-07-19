#pragma once

// c++
#include <vector>
#include <mutex>  

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>

// darknet_ros
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// Opencv
#include <cv_bridge/cv_bridge.h>

// Spencer
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>

// realsense_yolo
#include <realsense_yolo/debug_yolo.h>

namespace realsense_yolo{
    
    typedef struct bbox_t_3d{
        bbox_t_3d(darknet_ros_msgs::BoundingBox bbox,float coord)
            :m_bbox(bbox),m_coord(coord){}

        darknet_ros_msgs::BoundingBox m_bbox;
        float m_coord;
    } bbox_t_3d;

    typedef struct camera_info_vector{
        // camera_info_vector(float fx,float fy,float cx,float cy)
        //     :m_fx(fx),m_fy(fy),m_cx(cx),m_cy(cy){}
        float m_fx,m_fy,m_cx,m_cy;
    } camera_info_vector;

    class realsense_yolo_detector{
        public:
            // Constructor
            explicit realsense_yolo_detector(ros::NodeHandle nodeHandle_,const sensor_msgs::CameraInfo::ConstPtr& camera_info);
            // Destructor
            ~realsense_yolo_detector();
        
        private:
            ros::NodeHandle nodeHandle_;

	        ros::Publisher people_position_pub,bbox3d_pub,debug_yolo_pub;
            ros::Subscriber camera_info_sub;

            // Parameter for nh.param()
            std::string detection_output_pub, camera_link,marker_array_topic,depth_topic,camera_info;
            float probability_threshold;
            std::vector<std::string> obj_list;

            double LARGE_VARIANCE_;
            double pose_variance_;
            int detection_id_increment_, detection_id_offset_, current_detection_id_; // added for multi-sensor use in SPENCER
            
            // Global variable
            std::mutex data_lock;
            camera_info_vector intrinsic_camera_matrix;         // ?? why don't need constructor in the typedef struct 
            realsense_yolo::debug_yolo debug_message;

            message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> m_yolo_detection_result_sub;
            message_filters::Subscriber<sensor_msgs::Image> m_depth_img_sub;

            typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> SyncPolicy;
            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            boost::shared_ptr<Synchronizer> sync;
            
            // Function & Callback function
            void init();
            void filterOutUnwantedDetections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_detection_raw_result, const sensor_msgs::Image::ConstPtr& depth_image);
            void draw_boxes();
            void camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
            spencer_tracking_msgs::DetectedPersons fillPeopleMessage(std::vector<bbox_t_3d> result_vec, std::string obj_names, std::string camera_frame_id);
    };
}
