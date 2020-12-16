#include <realsense_yolo/realsense_yolo.hpp>

//Pipeline: filterOutUnwantedDetections -> getObjectDepth -> drawboxes -> fill message -> publish message

namespace realsense_yolo{

	realsense_yolo_detector::realsense_yolo_detector(ros::NodeHandle& nh,const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
		: nodeHandle_(nh){

		ROS_INFO("[Yolo 3D Object Detector with realsense D435] Node started.");
		
		this->init();
		this->camera_infoCallback(camera_info_msg);
	}
	
	realsense_yolo_detector::~realsense_yolo_detector(){
		ROS_INFO("Closing realsense_yolo_detector!!!");
		// ??: how to free the memory
		// if (it_ != 0) delete it_;
		// if (sync_input_2_ != 0) delete sync_input_2_;
		// if (add_data_server_ != 0) delete add_data_server_;
		// if (update_data_server_ != 0) delete update_data_server_;
		// if (delete_data_server_ != 0) delete delete_data_server_;
	}

	void realsense_yolo_detector::init(){
		
		ROS_INFO("[Yolo 3D Object Detector with realsense D435] init().");

		// parameters name , variable_name, default value
		nodeHandle_.param("depth_image_topic", depth_topic, std::string("/spencer/sensors/rgbd_front_top/depth/image_rect_raw"));

		// simulation change
		// nodeHandle_.param("pointcloud2_topic", pointcloud2_topic, std::string("/camera/depth/color/points"));
		// nodeHandle_.param("cameralink", camera_link, std::string("camera_depth_optical_frame"));
		nodeHandle_.param("pointcloud2_topic", pointcloud2_topic, std::string("/spencer/sensors/rgbd_front_top/depth_registered/points"));		
		nodeHandle_.param("cameralink", camera_link, std::string("rgbd_front_top_depth_optical_frame"));

		nodeHandle_.param("detection_output", detection_output_pub, std::string("/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body"));
		nodeHandle_.param("yolo_detection_threshold", probability_threshold,float(0.5));

		nodeHandle_.param("pose_variance", pose_variance_, 0.05);
     	nodeHandle_.param("detection_id_increment", detection_id_increment_, 1);
      	nodeHandle_.param("detection_id_offset",detection_id_offset_, 10000);
		nodeHandle_.param("marker_array_topic", marker_array_topic,std::string("/bbox3d_marker_array"));
		
		LARGE_VARIANCE_ = 999999999.0;
      	current_detection_id_ = detection_id_offset_;
		
		obj_list = {"person"};

		// https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
		// ??: Why cannot yolo_detection_result_sub.reset() to subscribe the new topic

		// Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
		m_yolo_detection_result_sub.subscribe(nodeHandle_,"/darknet_ros/bounding_boxes",1);
		m_depth_img_sub.subscribe(nodeHandle_,depth_topic , 1);
		m_pointcloud_sub.subscribe(nodeHandle_,pointcloud2_topic,1);
		sync.reset(new Synchronizer(SyncPolicy(10),m_yolo_detection_result_sub,m_depth_img_sub,m_pointcloud_sub));
		sync->registerCallback(boost::bind(&realsense_yolo_detector::filterOutUnwantedDetections, this,_1, _2,_3));

		people_position_pub = nodeHandle_.advertise<spencer_tracking_msgs::DetectedPersons>(detection_output_pub, 1);
		bbox3d_pub = nodeHandle_.advertise<visualization_msgs::MarkerArray>(marker_array_topic, 1);
		debug_yolo_pub = nodeHandle_.advertise<realsense_yolo::debug_yolo>("debug_yolo_topic", 1);

		image_transport::ImageTransport it(nodeHandle_);
		yolo_image_pub = it.advertise("detection_result", 1);

	}

	void realsense_yolo_detector::camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg){
		data_lock.lock();
		ROS_INFO("Camera Info get!!!");
		intrinsic_camera_matrix.m_fx=camera_info_msg->K[0];
		intrinsic_camera_matrix.m_fy=camera_info_msg->K[4];
		intrinsic_camera_matrix.m_cx=camera_info_msg->K[2];
		intrinsic_camera_matrix.m_cy=camera_info_msg->K[5];
		// ROS_INFO("%f %f %f %f",intrinsic_camera_matrix.m_fx,intrinsic_camera_matrix.m_fy,intrinsic_camera_matrix.m_cx,intrinsic_camera_matrix.m_cy);
		data_lock.unlock();

		// !! can try unsubscribe: 
	}

	void realsense_yolo_detector::filterOutUnwantedDetections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_detection_raw_result, 
				const sensor_msgs::Image::ConstPtr& depth_image,const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){
		// gettimeofday(&program_start,NULL);		
		int n = yolo_detection_raw_result->bounding_boxes.size();
		std::vector<darknet_ros_msgs::BoundingBox> filtered_detections;
		std::vector<int> erase_these_elements;
		std::vector<darknet_ros_msgs::BoundingBox> filtered_detections_wo_doubles;

		std::vector<bbox_t_3d> yolo_detection_xyz {};
		filtered_detections.reserve(n);
		erase_these_elements.reserve(n);
		filtered_detections_wo_doubles.reserve(n);
				
		if(n>0)
		{
			for(int i=0;i<n;i++){
				for(auto &it:obj_list){
					if(yolo_detection_raw_result->bounding_boxes[i].Class == it && 
					yolo_detection_raw_result->bounding_boxes[i].probability>probability_threshold){
						filtered_detections.push_back(yolo_detection_raw_result->bounding_boxes[i]);
					}
					// else{
					// 	ROS_ERROR("Debuggingggg!");
					// }
				}
			}


			// gettimeofday(&check_start,NULL);
			// Sometimes, there are two or even more bounding boxes per detected person. 
			// Use nonmax suppression to only keep the bounding box with the highest confidence	
			for (int i = 0; i < filtered_detections.size(); i++) {
				for (int j = 0; j < filtered_detections.size(); j++) {
					// check if one bounding box lies completely within the other one. If it does, only keep the box with the higher confidence
					if ( (j != i) && (
							(filtered_detections[i].xmin < filtered_detections[j].xmin && 
							filtered_detections[i].ymin  < filtered_detections[j].ymin &&
							filtered_detections[i].xmax  > filtered_detections[j].xmax &&
							filtered_detections[i].ymax  > filtered_detections[j].ymax) || 
							(filtered_detections[i].xmin > filtered_detections[j].xmin && 
							filtered_detections[i].ymin  > filtered_detections[j].ymin &&
							filtered_detections[i].xmax  < filtered_detections[j].xmax &&
							filtered_detections[i].ymax  < filtered_detections[j].ymax))) {
								
						if (filtered_detections[i].probability > filtered_detections[j].probability) {
							erase_these_elements.push_back(j);
						} else {
							erase_these_elements.push_back(i);
						}
					}
				}
			}

			// store all bounding boxes that lie not within each other
			for (int i = 0; i < filtered_detections.size(); i++) {
				if(std::find(erase_these_elements.begin(), erase_these_elements.end(), i) != erase_these_elements.end()) {
					
				} else {
					filtered_detections_wo_doubles.push_back(filtered_detections[i]);
				}
			}
			// gettimeofday(&check_stop,NULL);
			// ROS_ERROR("checking took %lu us\n", (check_stop.tv_sec - check_start.tv_sec) * 1000000 + check_stop.tv_usec - check_start.tv_usec);

			// ROS_INFO("Raw: %d --- Detected %lu person!",n,filtered_detections_wo_doubles.size());
			// debug_message.no_detected_person = filtered_detections_wo_doubles.size();
			// debug_message.no_deleted_detection = erase_these_elements.size();

		}
		
		/*----------------------- Get depth from depth image -----------------------*/
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(depth_image,sensor_msgs::image_encodings::TYPE_16UC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		// cv::Mat cvImage_depth;
		// cv::Mat draw_result;

		// cvImage_depth = cv_ptr->image;
		// draw_result = cvImage_depth.clone();

		for (int i = 0; i < filtered_detections_wo_doubles.size(); i++) {
			float pixel_distance;
			int pixel_height = (filtered_detections_wo_doubles[i].ymax+filtered_detections_wo_doubles[i].ymin)/2;
			int pixel_width = (filtered_detections_wo_doubles[i].xmax+filtered_detections_wo_doubles[i].xmin)/2;
		
			// TODO:  calculate the median of all the values inside the box region
			pixel_distance = 0.001*(cv_ptr->image.at<u_int16_t>(pixel_height,pixel_width));

			// cv::putText(draw_result,std::to_string(pixel_distance),cv::Point2f(pixel_width,pixel_height),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,143,143), 2);
			// cv::circle(draw_result,cv::Point2f(pixel_width+10,pixel_height),10,cv::Scalar(0),2);

			yolo_detection_xyz.emplace_back((bbox_t_3d(filtered_detections_wo_doubles[i],pixel_distance)));
		}

		// sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, draw_result).toImageMsg();
		// yolo_image_pub.publish(msg);
		
		/*-------------- Publish Spencer data from Yolo detection ---------*/
		// TODO: pass obj_list as vector into fillPeopleMessage()
		auto detected_persons_msg = this->fillPeopleMessage(yolo_detection_xyz,"person",camera_link,pointcloud_msg);
		people_position_pub.publish(detected_persons_msg);
		
		// if(boudingbox_pcl){
		// 	this->calculate_boxes_pcl(pointcloud_msg,yolo_detection_raw_result->bounding_boxes);
		// }
		// else{
		// 	this->calculate_boxes(yolo_detection_xyz);
		// }
		// gettimeofday(&program_stop,NULL);
		// ROS_ERROR("took %lu us\n", (program_stop.tv_sec - program_start.tv_sec) * 1000000 + program_stop.tv_usec - program_start.tv_usec);
	}

	// void realsense_yolo_detector::calculate_boxes_pcl(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg,
	// 		std::vector<darknet_ros_msgs::BoundingBox> original_bboxes_){
	// 	/*
	// 	Reference Link: https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d
	// 	*/
	// 	sensor_msgs::PointCloud2 local_pointcloud;
	// 	sensor_msgs::PointCloud2 point_cloud_ = *pointcloud_msg;

	// 	//API: http://docs.ros.org/kinetic/api/pcl_ros/html/namespacepcl__ros.html#a34090d5c8739e1a31749ccf0fd807f91
	// 	try{
	// 		pcl_ros::transformPointCloud(camera_link, point_cloud_, local_pointcloud, tfListener_);
  	// 	}
  	// 	catch(tf::TransformException& ex){
    // 		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
  	// 	}

	// 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  	// 	pcl::fromROSMsg(local_pointcloud, *pcrgb);

	// 	realsense_yolo::BoundingBoxes3d boxes;

  	// 	boxes.header.frame_id = camera_link;
	// 	boxes.header.stamp = point_cloud_.header.stamp;

		
	// 	if(pcrgb->size()>0){
	// 		for(auto bbx:original_bboxes_){

	// 			if ((bbx.probability < probability_threshold) ||
	// 				(std::find(obj_list.begin(), obj_list.end(), bbx.Class) == obj_list.end()))
	// 			{
	// 			continue;
	// 			}

	// 			int center_x, center_y;

	// 			center_x = (bbx.xmax + bbx.xmin) / 2;
	// 			center_y = (bbx.ymax + bbx.ymin) / 2;

	// 			int pcl_index = (center_y* pointcloud_msg->width) + center_x;

	// 			pcl::PointXYZRGB center_point =  pcrgb->at(pcl_index); 

	// 			if (std::isnan(center_point.x))
	// 				continue;

	// 			float maxx, minx, maxy, miny, maxz, minz;

	// 			maxx = maxy = maxz =  -std::numeric_limits<float>::max();
	// 			minx = miny = minz =  std::numeric_limits<float>::max();

	// 			for (int i = bbx.xmin; i < bbx.xmax; i++){
	// 				for (int j = bbx.ymin; j < bbx.ymax; j++){

	// 					pcl_index = (j* pointcloud_msg->width) + i;
	// 					pcl::PointXYZRGB point =  pcrgb->at(pcl_index);

	// 					if (std::isnan(point.x))
	// 						continue;

	// 					if (fabs(point.x - center_point.x) > probability_threshold)
	// 						continue;

	// 					maxx = std::max(point.x, maxx);
	// 					maxy = std::max(point.y, maxy);
	// 					maxz = std::max(point.z, maxz);
	// 					minx = std::min(point.x, minx);
	// 					miny = std::min(point.y, miny);
	// 					minz = std::min(point.z, minz);
	// 				}
	// 			}

	// 			realsense_yolo::BoundingBox3d bbx_msg;
	// 			bbx_msg.Class = bbx.Class;
	// 			bbx_msg.probability = bbx.probability;
	// 			bbx_msg.xmin = minx;
	// 			bbx_msg.xmax = maxx;
	// 			bbx_msg.ymin = miny;
	// 			bbx_msg.ymax = maxy;
	// 			bbx_msg.zmin = minz;
	// 			bbx_msg.zmax = maxz;

	// 			boxes.bounding_boxes.push_back(bbx_msg); 
	// 		}

	// 		this->draw_boxes(boxes);
	// 	}
	// }

	void realsense_yolo_detector::calculate_boxes(std::vector<bbox_t_3d> result_vec){
		realsense_yolo::BoundingBoxes3d boxes;

  		boxes.header.frame_id = camera_link;
		// boxes.header.stamp = point_cloud_.header.stamp;

		for (auto &i : result_vec) {
			realsense_yolo::BoundingBox3d bbx_msg;
			bbx_msg.Class = i.m_bbox.Class;
			bbx_msg.probability = i.m_bbox.probability;
			bbx_msg.xmin = i.m_bbox.xmin;
			bbx_msg.xmax = i.m_bbox.xmax;
			bbx_msg.ymin = i.m_bbox.ymin;
			bbx_msg.ymax = i.m_bbox.ymax;
			bbx_msg.zmin = 0.5*i.m_coord;
			bbx_msg.zmax = i.m_coord;
			boxes.bounding_boxes.push_back(bbx_msg);
		}

		this->draw_boxes(boxes);
	}

	void realsense_yolo_detector::draw_boxes(const realsense_yolo::BoundingBoxes3d boxes){
		// publish 3D bounding box to rviz for visualization
		visualization_msgs::MarkerArray msg;

		int counter_id = 0;
		for (auto bb : boxes.bounding_boxes)
		{
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

			if(boudingbox_pcl){
				bbx_marker.type = visualization_msgs::Marker::CUBE;

				bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
				bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
				bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;
				
				bbx_marker.scale.x = (bb.xmax - bb.xmin);
				bbx_marker.scale.y = (bb.ymax - bb.ymin);
				bbx_marker.scale.z = (bb.zmax - bb.zmin);
			}

			else{
				bbx_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

				bbx_marker.scale.x = 1;
				bbx_marker.scale.y = 1;
				bbx_marker.scale.z = 1;

				bbx_marker.pose.position.x = ((bb.xmin+bb.xmax)/2-intrinsic_camera_matrix.m_cx)*(bb.zmax/intrinsic_camera_matrix.m_fx);
				bbx_marker.pose.position.y = ((bb.ymin+bb.ymax)/2-intrinsic_camera_matrix.m_cy)*(bb.zmax/intrinsic_camera_matrix.m_fy);
				bbx_marker.pose.position.z = bb.zmax;

				bbx_marker.mesh_resource = "package://spencer_tracking_rviz_plugin/media/animated_walking_man.mesh";
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

	spencer_tracking_msgs::DetectedPersons realsense_yolo_detector::fillPeopleMessage(
			std::vector<bbox_t_3d> result_vec, 
			std::string obj_names, 
			std::string camera_frame_id,
			const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){

		spencer_tracking_msgs::DetectedPersons  detected_persons;

		data_lock.lock();

		bool distance_from_pointcloud= true;
		std::vector<float> person_xyz;
		for (auto &i : result_vec) {
			/*
			Pixel convert to real distance:
			(i) Intrinsic Camera Calibration
			https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect

			(ii) PointCloud from PointCloud2 to get xyz
			https://answers.ros.org/question/9239/reading-pointcloud2-in-c/
			*/
			bool nan_value = false;

			int x_center,y_center;
			float Xreal,Yreal,Zreal;

			spencer_tracking_msgs::DetectedPerson  detected_person;
			
			sensor_msgs::PointCloud out_cloud;
			sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud_msg, out_cloud);

			x_center = (i.m_bbox.xmin+i.m_bbox.xmax)/2;
			y_center = (i.m_bbox.ymin+i.m_bbox.ymax)/2;

			int ind = (x_center) + (y_center)*pointcloud_msg->width;

			Xreal = (float)out_cloud.points[ind].x;
			Yreal = (float)out_cloud.points[ind].y;
			Zreal = (float)out_cloud.points[ind].z;
			
			if (distance_from_pointcloud){
				// ??: hardcode the transformation for position x, need to find out why it is wrong
				detected_person.pose.pose.position.x = Xreal;
				detected_person.pose.pose.position.y = Yreal;
				detected_person.pose.pose.position.z = Zreal;
			}
			else{
				detected_person.pose.pose.position.x = ((i.m_bbox.xmin+i.m_bbox.xmax)/2-intrinsic_camera_matrix.m_cx)*(i.m_coord/intrinsic_camera_matrix.m_fx);
				detected_person.pose.pose.position.y = ((i.m_bbox.ymin+i.m_bbox.ymax)/2-intrinsic_camera_matrix.m_cy)*(i.m_coord/intrinsic_camera_matrix.m_fy);
				detected_person.pose.pose.position.z = i.m_coord;
			}

			if(isnan(detected_person.pose.pose.position.x)|isnan(detected_person.pose.pose.position.y)|isnan(detected_person.pose.pose.position.z)){
				nan_value = true;
			}
			
			detected_person.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_RGBD;
			// detected_person.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_YOLO;
			detected_person.confidence = i.m_bbox.probability;

			// ??: pose_variance
			detected_person.pose.covariance[0*6 + 0] = pose_variance_;
            detected_person.pose.covariance[1*6 + 1] = pose_variance_;
            detected_person.pose.covariance[2*6 + 2] = pose_variance_;
            detected_person.pose.covariance[3*6 + 3] = LARGE_VARIANCE_;
            detected_person.pose.covariance[4*6 + 4] = LARGE_VARIANCE_;
            detected_person.pose.covariance[5*6 + 5] = LARGE_VARIANCE_;

            detected_person.detection_id = current_detection_id_;
            current_detection_id_ += detection_id_increment_;
			
			if(!nan_value){
				detected_persons.detections.push_back(detected_person);
			
				person_xyz.push_back(detected_person.pose.pose.position.x);
				person_xyz.push_back(detected_person.pose.pose.position.y);
				person_xyz.push_back(detected_person.pose.pose.position.z);
			}
   		}

		// debug_message.person_xyz = person_xyz;

		data_lock.unlock();

		detected_persons.header.stamp = ros::Time::now();
		detected_persons.header.frame_id = camera_frame_id;
		
		// debug_yolo_pub.publish(debug_message);
		
		return detected_persons;
		}

} /* namespace realsense_yolo */
