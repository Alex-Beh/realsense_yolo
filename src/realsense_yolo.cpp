#include <realsense_yolo/realsense_yolo.hpp>

// Pipeline: filterOutUnwantedDetections -> getObjectDepth -> drawboxes -> fill
// message -> publish message

namespace realsense_yolo {

realsense_yolo_detector::realsense_yolo_detector(
    ros::NodeHandle &nh,
    const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg)
    : nodeHandle_(nh) {

  ROS_INFO("[Yolo 3D Object Detector with realsense D435] Node started.");

  this->init();
  this->camera_infoCallback(camera_info_msg);
}

realsense_yolo_detector::~realsense_yolo_detector() {
  ROS_INFO("Closing realsense_yolo_detector!!!");
  // ??: how to free the memory
  // if (it_ != 0) delete it_;
  // if (sync_input_2_ != 0) delete sync_input_2_;
  // if (add_data_server_ != 0) delete add_data_server_;
  // if (update_data_server_ != 0) delete update_data_server_;
  // if (delete_data_server_ != 0) delete delete_data_server_;
}

void realsense_yolo_detector::init() {

  ROS_INFO("[Yolo 3D Object Detector with realsense D435] init().");

  // parameters name , variable_name, default value
  nodeHandle_.param(
      "depth_image_topic", depth_topic,
      std::string("/spencer/sensors/rgbd_front_top/aligned_depth_to_color/image_raw"));

  ///1234 
  // nodeHandle_.param(
  //   "depth_image_topic", depth_topic,
  //   std::string("/camera/aligned_depth_to_color/image_raw"));

  nodeHandle_.param("cameralink", camera_link,
                    std::string("rgbd_front_top_depth_optical_frame"));

  nodeHandle_.param("detection_output", detection_output_pub,
                    std::string("/spencer/perception_internal/detected_persons/"
                                "rgbd_front_top/upper_body"));
  nodeHandle_.param("yolo_detection_threshold", probability_threshold,
                    float(0.5));

  nodeHandle_.param("pose_variance", pose_variance_, 0.05);
  nodeHandle_.param("detection_id_increment", detection_id_increment_, 1);
  nodeHandle_.param("detection_id_offset", detection_id_offset_, 10000);
  nodeHandle_.param("marker_array_topic", marker_array_topic,
                    std::string("/bbox3d_marker_array"));

  LARGE_VARIANCE_ = 999999999.0;
  current_detection_id_ = detection_id_offset_;

  obj_list = {"person"};

  // Set queue size to 1 because generating a queue here will only pile up
  // images and delay the output by the amount of queued images
  m_yolo_detection_result_sub.subscribe(nodeHandle_,
                                        "/darknet_ros/bounding_boxes", 1);
  m_depth_img_sub.subscribe(nodeHandle_, depth_topic, 1);


  sync.reset(new Synchronizer(SyncPolicy(10), m_yolo_detection_result_sub,
                              m_depth_img_sub));
  sync->registerCallback(boost::bind(
      &realsense_yolo_detector::filterOutUnwantedDetections, this, _1, _2));

#ifdef WITH_SPENCER
  people_position_pub =
      nodeHandle_.advertise<spencer_tracking_msgs::DetectedPersons>(
          detection_output_pub, 1);
#endif

  bbox3d_pub = nodeHandle_.advertise<visualization_msgs::MarkerArray>(
      marker_array_topic, 1);
}

void realsense_yolo_detector::camera_infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg) {
  ROS_INFO("Camera Info get!!!");
  intrinsic_camera_matrix.m_fx = camera_info_msg->K[0];
  intrinsic_camera_matrix.m_fy = camera_info_msg->K[4];
  intrinsic_camera_matrix.m_cx = camera_info_msg->K[2];
  intrinsic_camera_matrix.m_cy = camera_info_msg->K[5];
  // ROS_INFO("%f %f %f
  // %f",intrinsic_camera_matrix.m_fx,intrinsic_camera_matrix.m_fy,intrinsic_camera_matrix.m_cx,intrinsic_camera_matrix.m_cy);
}

void realsense_yolo_detector::filterOutUnwantedDetections(
    const realsense_yolo::BoundingBoxes3d::ConstPtr &yolo_detection_raw_result,
    const sensor_msgs::Image::ConstPtr &depth_image) {
  currentTime = depth_image->header.stamp.toSec();

  int n = yolo_detection_raw_result->bounding_boxes.size();
  std::vector<realsense_yolo::BoundingBox3d> filtered_detections;
  std::vector<realsense_yolo::BoundingBox3d> filtered_detections_wo_doubles;
  std::vector<int> erase_these_elements;

  filtered_detections.reserve(n);
  filtered_detections_wo_doubles.reserve(n);
  erase_these_elements.reserve(n);
  std::vector<bbox_t_3d> yolo_detection_xyz{};

  if (n > 0) {
    for (int i = 0; i < n; i++) {
      for (auto &it : obj_list) {
        if (yolo_detection_raw_result->bounding_boxes[i].Class == it &&
            yolo_detection_raw_result->bounding_boxes[i].probability >
                probability_threshold) {

          filtered_detections.push_back(
              yolo_detection_raw_result->bounding_boxes[i]);
        }
      }
    }

    // Use nonmax suppression to only keep the bounding box with the highest
    // confidence Use nonmax suppression to only keep the bounding box with the
    // highest confidence
    // Use nonmax suppression to only keep the bounding box with the highest
    // confidence
    for (int i = 0; i < filtered_detections.size(); i++) {
      for (int j = 0; j < filtered_detections.size(); j++) {
        // check if one bounding box lies completely within the other one. If it
        // does, only keep the box with the higher confidence
        if ((j != i) &&
            ((filtered_detections[i].xmin < filtered_detections[j].xmin &&
              filtered_detections[i].ymin < filtered_detections[j].ymin &&
              filtered_detections[i].xmax > filtered_detections[j].xmax &&
              filtered_detections[i].ymax > filtered_detections[j].ymax) ||
             (filtered_detections[i].xmin > filtered_detections[j].xmin &&
              (filtered_detections[i].xmin > filtered_detections[j].xmin &&
               (filtered_detections[i].xmin > filtered_detections[j].xmin &&
                filtered_detections[i].ymin > filtered_detections[j].ymin &&
                filtered_detections[i].xmax < filtered_detections[j].xmax &&
                filtered_detections[i].ymax < filtered_detections[j].ymax))))) {
          if (filtered_detections[i].probability >
              filtered_detections[j].probability) {
            erase_these_elements.push_back(j);
          } else {
            erase_these_elements.push_back(i);
          }
        }
      }
    }

    // store all bounding boxes that lie not within each other
    for (int i = 0; i < filtered_detections.size(); i++) {
      if (std::find(erase_these_elements.begin(), erase_these_elements.end(),
                    i) != erase_these_elements.end()) {
      } else {
        filtered_detections_wo_doubles.push_back(filtered_detections[i]);
      }
    }
  }

  /*----------------------- Get depth from depth image -----------------------*/
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(depth_image,
                                 sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  for (int i = 0; i < filtered_detections_wo_doubles.size(); i++) {
    float pixel_distance;
    int pixel_height = (filtered_detections_wo_doubles[i].ymax +
                        filtered_detections_wo_doubles[i].ymin) /
                       2;
    int pixel_width = (filtered_detections_wo_doubles[i].xmax +
                       filtered_detections_wo_doubles[i].xmin) /
                      2;

    // TODO:  calculate the median of all the values inside the box region
    pixel_distance =
        0.001 * (cv_ptr->image.at<u_int16_t>(pixel_height, pixel_width));

    if (pixel_distance<0.2 || pixel_distance>8) continue;
    
    yolo_detection_xyz.emplace_back(
        (bbox_t_3d(filtered_detections_wo_doubles[i], pixel_distance)));
    
    // if(pixel_distance<0.2 || pixel_distance>8){
    //   ROS_WARN("Wrong liao Wrong liao Pixel(%d,%d) --- Distance: %f",pixel_height,pixel_width,pixel_distance);
    // }
    // else{
    //   ROS_INFO("Pixel(%d,%d) --- Distance: %f",pixel_height,pixel_width,pixel_distance);
    // }
  }

/*-------------- Publish Spencer data from Yolo detection ---------*/
// TODO: pass obj_list as vector into fillPeopleMessage()
#ifdef WITH_SPENCER
  auto detected_persons_msg = this->fillPeopleMessage(
      yolo_detection_xyz, "person", camera_link);
  people_position_pub.publish(detected_persons_msg);
#endif
  this->calculate_boxes(yolo_detection_xyz);
}

#ifdef WITH_SPENCER
spencer_tracking_msgs::DetectedPersons
realsense_yolo_detector::fillPeopleMessage(
    std::vector<bbox_t_3d> result_vec, std::string obj_names,
    std::string camera_frame_id) {

  spencer_tracking_msgs::DetectedPersons detected_persons;

  for (auto &i : result_vec) {
    
    spencer_tracking_msgs::DetectedPerson detected_person;

    detected_person.pose.pose.position.x =
        ((i.m_bbox.xmin + i.m_bbox.xmax) / 2 - intrinsic_camera_matrix.m_cx) *
        (i.m_coord / intrinsic_camera_matrix.m_fx);
    detected_person.pose.pose.position.y =
        ((i.m_bbox.ymin + i.m_bbox.ymax) / 2 - intrinsic_camera_matrix.m_cy) *
        (i.m_coord / intrinsic_camera_matrix.m_fy);
    detected_person.pose.pose.position.z = i.m_coord;

    if (!(isnan(detected_person.pose.pose.position.x) |
          isnan(detected_person.pose.pose.position.y) |
          isnan(detected_person.pose.pose.position.z))) {

      detected_person.modality =
          spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_RGBD;
      detected_person.confidence = i.m_bbox.probability;

      detected_person.pose.covariance[0 * 6 + 0] = pose_variance_;
      detected_person.pose.covariance[1 * 6 + 1] = pose_variance_;
      detected_person.pose.covariance[2 * 6 + 2] = LARGE_VARIANCE_;
      detected_person.pose.covariance[3 * 6 + 3] = LARGE_VARIANCE_;
      detected_person.pose.covariance[4 * 6 + 4] = LARGE_VARIANCE_;
      detected_person.pose.covariance[5 * 6 + 5] = LARGE_VARIANCE_;

      detected_person.detection_id = current_detection_id_;
      current_detection_id_ += detection_id_increment_;

      detected_persons.detections.push_back(detected_person);
    }
  }
  detected_persons.header.stamp = ros::Time(currentTime);
  detected_persons.header.frame_id = camera_frame_id;

  return detected_persons;
}
#endif

void realsense_yolo_detector::calculate_boxes(
    std::vector<bbox_t_3d> result_vec) {
  realsense_yolo::BoundingBoxes3d boxes;

  boxes.header.frame_id = camera_link;

  for (auto &i : result_vec) {
    realsense_yolo::BoundingBox3d bbx_msg;
    bbx_msg.Class = i.m_bbox.Class;
    bbx_msg.probability = i.m_bbox.probability;
    bbx_msg.xmin = i.m_bbox.xmin;
    bbx_msg.xmax = i.m_bbox.xmax;
    bbx_msg.ymin = i.m_bbox.ymin;
    bbx_msg.ymax = i.m_bbox.ymax;
    bbx_msg.zmin = 0.5 * i.m_coord;
    bbx_msg.zmax = i.m_coord;
    boxes.bounding_boxes.push_back(bbx_msg);
  }

  this->draw_boxes(boxes);
}

void realsense_yolo_detector::draw_boxes(
    const realsense_yolo::BoundingBoxes3d boxes) {
  // publish 3D bounding box to rviz for visualization
  visualization_msgs::MarkerArray msg;

  int counter_id = 0;
  for (auto bb : boxes.bounding_boxes) {
    visualization_msgs::Marker bbx_marker;

    bbx_marker.header.frame_id = boxes.header.frame_id;
    bbx_marker.header.stamp = boxes.header.stamp;
    bbx_marker.ns = "darknet3d";
    bbx_marker.id = counter_id++;

    bbx_marker.action = visualization_msgs::Marker::ADD;

    bbx_marker.pose.orientation.x = 0.0;
    bbx_marker.pose.orientation.y = 0.0;
    bbx_marker.pose.orientation.z = 0.0;
    bbx_marker.pose.orientation.w = 1.0;

    if (boudingbox_pcl) {
      bbx_marker.type = visualization_msgs::Marker::CUBE;

      bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
      bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
      bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;

      bbx_marker.scale.x = (bb.xmax - bb.xmin);
      bbx_marker.scale.y = (bb.ymax - bb.ymin);
      bbx_marker.scale.z = (bb.zmax - bb.zmin);
    }

    else {
      bbx_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

      bbx_marker.scale.x = 1;
      bbx_marker.scale.y = 1;
      bbx_marker.scale.z = 1;

      bbx_marker.pose.position.x =
          ((bb.xmin + bb.xmax) / 2 - intrinsic_camera_matrix.m_cx) *
          (bb.zmax / intrinsic_camera_matrix.m_fx);
      bbx_marker.pose.position.y =
          ((bb.ymin + bb.ymax) / 2 - intrinsic_camera_matrix.m_cy) *
          (bb.zmax / intrinsic_camera_matrix.m_fy);
      bbx_marker.pose.position.z = bb.zmax;

      bbx_marker.mesh_resource = "package://realsense_yolo/media/"
                                 "animated_walking_man.mesh";
    }

    bbx_marker.color.b = 0;
    bbx_marker.color.g = bb.probability * 255.0;
    bbx_marker.color.r = (1.0 - bb.probability) * 255.0;
    bbx_marker.color.a = 0.4;
    bbx_marker.lifetime = ros::Duration(0.5);

    msg.markers.push_back(bbx_marker);
  }

  bbox3d_pub.publish(msg);
}

} /* namespace realsense_yolo */
