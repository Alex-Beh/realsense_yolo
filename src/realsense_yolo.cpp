#include <realsense_yolo/realsense_yolo.hpp>

//Pipelin: filterOutUnwantedDetections -> getObjectDepth -> drawboxes -> fill message -> publish message

namespace realsense_yolo{

	realsense_yolo_detector::realsense_yolo_detector(ros::NodeHandle nh,const sensor_msgs::CameraInfo::ConstPtr& camera_info)
		: nodeHandle_(nh){

		ROS_INFO("[Yolo 3D Object Detector with realsense D435] Node started.");

		this->init();
		this->camera_infoCallback(camera_info);
	}
	
	realsense_yolo_detector::~realsense_yolo_detector(){
		ROS_INFO("Closing realsense_yolo_detector!!!");
		// free the memory
	}

	void realsense_yolo_detector::init(){
		
		ROS_INFO("[Yolo 3D Object Detector with realsense D435] init().");

		// parameters name , variable_name, default value
		nodeHandle_.param("camera_info", camera_info, std::string("/spencer/sensors/rgbd_front_top/depth/camera_info"));
		nodeHandle_.param("depth_image_topic", depth_topic, std::string("/spencer/sensors/rgbd_front_top/depth/image_rect_raw"));

		nodeHandle_.param("detection_output", detection_output_pub, std::string("/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body"));
		nodeHandle_.param("yolo_detection_threshold", probability_threshold,float(0.5));
		
		nodeHandle_.param("cameralink", camera_link, std::string("rgbd_front_top_link"));

		nodeHandle_.param("pose_variance", pose_variance_, 0.05);
     	nodeHandle_.param("detection_id_increment", detection_id_increment_, 1);
      	nodeHandle_.param("detection_id_offset",detection_id_offset_, 10000);
		nodeHandle_.param("marker_array_topic", marker_array_topic,std::string("/bbox3d_marker_array"));
		
		LARGE_VARIANCE_ = 999999999.0;
      	current_detection_id_ = detection_id_offset_;
		
		obj_list = {"person"};

		// https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
		// ??? Why cannot yolo_detection_result_sub.reset() to subscribe the new topic ???
		m_yolo_detection_result_sub.subscribe(nodeHandle_,"/darknet_ros/bounding_boxes",100);
		m_depth_img_sub.subscribe(nodeHandle_,depth_topic , 100);
		m_pointcloud_sub.subscribe(nodeHandle_,"/spencer/sensors/rgbd_front_top/depth_registered/points",100);
		sync.reset(new Synchronizer(SyncPolicy(10),m_yolo_detection_result_sub,m_depth_img_sub,m_pointcloud_sub));
		sync->registerCallback(boost::bind(&realsense_yolo_detector::filterOutUnwantedDetections, this,_1, _2,_3));

		people_position_pub = nodeHandle_.advertise<spencer_tracking_msgs::DetectedPersons>(detection_output_pub, 1);
		// camera_info_sub = nodeHandle_.subscribe<sensor_msgs::CameraInfo>(camera_info,1,&realsense_yolo_detector::camera_infoCallback,this);
		bbox3d_pub = nodeHandle_.advertise<visualization_msgs::MarkerArray>(marker_array_topic, 1);
		debug_yolo_pub = nodeHandle_.advertise<realsense_yolo::debug_yolo>("debug_yolo_topic", 1);

		image_transport::ImageTransport it(nodeHandle_);
		yolo_image_pub = it.advertise("detection_result", 1);

	}

	void realsense_yolo_detector::draw_boxes(){
		// ROS_INFO("To be done! No 3D bounging box now!");
		// publish to rviz for visualization
	}

	void realsense_yolo_detector::camera_infoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info){
		data_lock.lock();
		ROS_INFO("Camera Info get!!!");
		intrinsic_camera_matrix.m_fx=camera_info->K[0];
		intrinsic_camera_matrix.m_fy=camera_info->K[4];
		intrinsic_camera_matrix.m_cx=camera_info->K[2];
		intrinsic_camera_matrix.m_cy=camera_info->K[5];
		// ROS_INFO("%f %f %f %f",intrinsic_camera_matrix.m_fx,intrinsic_camera_matrix.m_fy,intrinsic_camera_matrix.m_cx,intrinsic_camera_matrix.m_cy);
		data_lock.unlock();
	}

	void realsense_yolo_detector::filterOutUnwantedDetections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_detection_raw_result, 
				const sensor_msgs::Image::ConstPtr& depth_image,const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){
		
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
			for(int i=i;i<n;i++){
				for(auto &it:obj_list){
					if(yolo_detection_raw_result->bounding_boxes[i].Class == it && yolo_detection_raw_result->bounding_boxes[i].probability>probability_threshold){
						filtered_detections.push_back(yolo_detection_raw_result->bounding_boxes[i]);
					}
				}
			}

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
		
			// ROS_INFO("Detected %lu person!",filtered_detections_wo_doubles.size());
			debug_message.no_detected_person = filtered_detections_wo_doubles.size();
			debug_message.no_deleted_detection = erase_these_elements.size();

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

		// TODO: check for cv_bridge fail case
		/* (row,col) --> (height,width) --> (640,480)
		    cvImage_depth.rows=480 , cvImage_depth.cols = 640
		   (cvImage_depth.rows,cvImage_depth.cols)
		*/
		cv::Mat cvImage_depth;
		cv::Mat draw_result;

		cvImage_depth = cv_ptr->image;
		draw_result = cvImage_depth.clone();

		for (int i = 0; i < filtered_detections_wo_doubles.size(); i++) {
			float pixel_distance;
			int pixel_height = (filtered_detections_wo_doubles[i].ymax+filtered_detections_wo_doubles[i].ymin)/2;
			int pixel_width = (filtered_detections_wo_doubles[i].xmax+filtered_detections_wo_doubles[i].xmin)/2;
		
			// TODO:  calculate the median of all the values inside the box region
			pixel_distance = 0.001*(cv_ptr->image.at<u_int16_t>(pixel_height,pixel_width));

			cv::putText(draw_result,std::to_string(pixel_distance),cv::Point2f(pixel_width,pixel_height),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,143,143), 2);
			cv::circle(draw_result,cv::Point2f(pixel_width+10,pixel_height),10,cv::Scalar(0),2);

			yolo_detection_xyz.emplace_back((bbox_t_3d(filtered_detections_wo_doubles[i],pixel_distance)));
		}
			this->draw_boxes();

			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, draw_result).toImageMsg();
			yolo_image_pub.publish(msg);
			
			/*-------------- Publish Spencer data from Yolo detection ---------*/
			// TODO: pass obj_list as vector into fillPeopleMessage()
			auto detected_persons_msg = this->fillPeopleMessage(yolo_detection_xyz,"person",camera_link,pointcloud_msg);
			people_position_pub.publish(detected_persons_msg);
		
	}

	spencer_tracking_msgs::DetectedPersons realsense_yolo_detector::fillPeopleMessage(
			std::vector<bbox_t_3d> result_vec, 
			std::string obj_names, 
			std::string camera_frame_id,
			const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){

		spencer_tracking_msgs::DetectedPersons  detected_persons;
		spencer_tracking_msgs::DetectedPerson  detected_person;

		// int array_length = 3*result_vec.size();
		// float xyz_array[array_length];
		// int array_pointer = 0;
		data_lock.lock();

		for (auto &i : result_vec) {
			/*
			Pixel convert to real distance:
			(i) Intrinsic Camera Calibration
			https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect

			X = (u - cx) * Z / fx;
    		Y = (v - cy) * Z / fy;

			u: 412 --- v: 263
			fx fy cx cy ---> 390.104401 390.104401 319.192230 241.850632

			***************
			x: 0.174982890487
			y: 0.039034537971
			z: 0.72000002861

			(ii) PointCloud from PointCloud2 to get xyz
			https://answers.ros.org/question/9239/reading-pointcloud2-in-c/
			*/
			bool distance_from_pointcloud= true;

			int x_center,y_center;
			float Xreal,Yreal,Zreal;
			sensor_msgs::PointCloud out_cloud;
			sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud_msg, out_cloud);

			x_center = (i.m_bbox.xmin+i.m_bbox.xmax)/2;
			y_center = (i.m_bbox.ymin+i.m_bbox.ymax)/2;

			int ind = (x_center) + (y_center)*pointcloud_msg->width;

			Xreal = (float)out_cloud.points[ind].x;
			Yreal = (float)out_cloud.points[ind].y;
			Zreal = (float)out_cloud.points[ind].z;
			
			if (distance_from_pointcloud){
				detected_person.pose.pose.position.x = Xreal;
				detected_person.pose.pose.position.y = Yreal;
				detected_person.pose.pose.position.z = Zreal;
			}
			else{
				detected_person.pose.pose.position.x = ((i.m_bbox.xmin+i.m_bbox.xmax)/2-intrinsic_camera_matrix.m_cx)*(i.m_coord/intrinsic_camera_matrix.m_fx);
				detected_person.pose.pose.position.y = ((i.m_bbox.ymin+i.m_bbox.ymax)/2-intrinsic_camera_matrix.m_cy)*(i.m_coord/intrinsic_camera_matrix.m_fy);
				detected_person.pose.pose.position.z = i.m_coord;
			}
			detected_person.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_YOLO;
			detected_person.confidence = i.m_bbox.probability;

			// ROS_INFO("			u: %lu --- v: %lu",(i.m_bbox.xmin+i.m_bbox.xmax)/2,(i.m_bbox.ymin+i.m_bbox.ymax)/2);
			// ROS_INFO("Intrinsic		x: %lf --- y: %lf --- z:%lf",detected_person.pose.pose.position.x,detected_person.pose.pose.position.y,detected_person.pose.pose.position.z);
			// ROS_INFO("PCL			x: %f --- y: %f --- z:%f",Xreal,Yreal,Zreal);

			// TODO: pose_variance???
			detected_person.pose.covariance[0*6 + 0] = pose_variance_;
            detected_person.pose.covariance[1*6 + 1] = pose_variance_;
            detected_person.pose.covariance[2*6 + 2] = pose_variance_;
            detected_person.pose.covariance[3*6 + 3] = LARGE_VARIANCE_;
            detected_person.pose.covariance[4*6 + 4] = LARGE_VARIANCE_;
            detected_person.pose.covariance[5*6 + 5] = LARGE_VARIANCE_;

            detected_person.detection_id = current_detection_id_;
            current_detection_id_ += detection_id_increment_;
			
			detected_persons.detections.push_back(detected_person);

			// ??? how to assign an array to ROS msg array type
			// debug_message.person_xyz[array_pointer++] = detected_person.pose.pose.position.x;
			// debug_message.person_xyz[array_pointer++] = detected_person.pose.pose.position.y;
			// debug_message.person_xyz[array_pointer++] = detected_person.pose.pose.position.z;
   		}
		// debug_message.person_xyz = xyz_array;

		data_lock.unlock();

		detected_persons.header.stamp = ros::Time::now();
		detected_persons.header.frame_id = camera_frame_id;
		
		debug_yolo_pub.publish(debug_message);
		
		return detected_persons;
		}

} /* namespace realsense_yolo */
