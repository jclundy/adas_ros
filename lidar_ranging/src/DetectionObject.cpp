/*
bool frame_detected = false;
int no_detection_count = 0;
double previous_distance = 0;
bool frame_has_appeared = false;

double prev_range_rate = 0;
double prev_range = 0;
double predicted_range = 0;
std::vector<double> measurements(MEASUREMENT_LIST_LENGTH);
int measurements_index = 0;
int measurement_count = 0;
*/


class DetectionObject
{
  public:
  // image frame variables
  int id;
  int cx;
  int cy;
  int frame_width;
  int frame_height;
  // detection state
  bool frame_detected;
  bool frame_has_appeared;
  int no_detection_count;
  // image ray (ie 3d vector) variables
  cv::Point3d ray;
  double azimuth;
  double elevation;
  // range estimation variables
  double range;
  double prev_range;
  double range_rate;
  double prev_range_rate;
  double lateral_range;
  int relative_lane;
  std::vector<double> measurements;
  int measurements_index;
  int measurement_count;
  
  DetectionObject()
  {
    id = -1;
    cx = 0;
    cy = 0;
    frame_width = 0;
    frame_height = 0;
    // detection state
    frame_detected = false;
    frame_has_appeared = false;
    no_detection_count = 0;
    // image ray (ie 3d vector) variables
    ray = cv::Point3d(0, 0, 0);
    azimuth = 0;
    elevation = 0;
    // range estimation variables
    range = 0;
    prev_range = 0;
    range_rate = 0;
    prev_range_rate = 0;
    lateral_range = 0;
    relative_lane = 0;
    measurements_index = 0;
    measurement_count = 0;
  }

  void update_detection_state(bool frame_detected)
  {
    frame_detected = (frame_detected_msg.data == true);
	  if(frame_detected)
	  {
		  frame_has_appeared = true;
	  } else if(frame_has_appeared) {
		  no_detection_count++;
		  if(no_detection_count > MAX_NO_DETECTION_COUNT)
		  {
			  std::printf("lost the detection\n");
			  frame_has_appeared = false;
			  no_detection_count = 0;
			  measurement_count = 0;
			  measurements_index = 0;
			  prev_range = 0;
        relative_lane = 0;
		  }
	  }
  }
  
  void update_detection_frame(int x, int y)
  {
    if(frame_detected)
	  {
      cx = x;
		  cy = y;
    }
  }

  void update_ray()
  {
    cv::Point2d frame_center = cv::Point2d(cx, cy);
	  cv::Point3d camera_ray = cam_model_.projectPixelTo3dRay(frame_center);
	
	  // convert camera coordinate system to vlp-16 coordinate system convention
	  double ray_world_x = camera_ray.z;
	  double ray_world_z = camera_ray.y;
	  double ray_world_y = camera_ray.x;

	  // apply rotation
	  ray.x = std::cos(theta_y)*ray_world_x + std::sin(theta_y)*ray_world_z;
	  ray.z = -std::sin(theta_y)*ray_world_x + std::cos(theta_y)*ray_world_z;
	  ray.y = ray_world_y;
    
    //update azimuth and bearing    
    azimuth = ray.y / ray.x
    bearing = ray.y / ray.x;
  }
  
  void update_lateral_range()
  {
    lateral_range = std::sin(azimuth) * range_estimation
  }
  
  void update_relative_lane()
  {
    relative_lane = 2;
    if(lateral_range < -2)
		  relative_lane = 1;
	  else if(lateral_range > 2)
		  relative_lane = 3;
  }
    
  void estimate_range(cv::Point3d camera_position, pcl::PointXYZ origin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
  {
    double measured_range = estimate_distance(ray, prev_range, prev_range_rate, camera_position, origin, cloud_filtered);
    range = measured_range;
    range_rate = (measured_range - prev_range) * LIDAR_DATA_RATE_HZ;

    measurements_index = (measurements_index + 1) % MEASUREMENT_LIST_LENGTH;
    measurements[measurements_index] = measured_range;
    if(measurement_count < MEASUREMENT_LIST_LENGTH && measured_range != 0)
    {
	    measurement_count++;
    }

    filter_range_estimation();
    prev_range_rate = range_rate;
    prev_range = range_estimation;
    
    update_lateral_range();
    update_relative_lane();
  }

  geometry_msgs::Point calculate_projected_point()
  {
    geometry_msgs::Point point;
		point.x = range * std::cos(azimuth) * std::cos(elevation);
		point.y = range * std::cos(elevation) * std::sin(azimuth);
		point.z = range * std::sin(elevation);		
		return point;
  }
  
  private:
  void filter_range_estimation()
  {
	  if(prev_range == 0) {
		  range_rate = DEFAULT_RANGE_RATE;
	  }
	  double predicted_range = prev_range + prev_range_rate * LIDAR_DATA_PERIOD_S;

	  if(range_rate != 0)
	  {
		  double average_of_prev_measurements = calculate_average(measurements, measurement_count);
		  double measurement_diff_with_average = abs(average_of_prev_measurements - measured_range);
		  double diff_bt_predicted_and_measured = std::abs(predicted_range - measured_range);

		  if(std::abs(range_rate) > MAX_RANGE_RATE)
		  {
			  if(measurement_count >= MEASUREMENT_LIST_LENGTH)
			  {
				  if(measurement_diff_with_average < RANGE_MEASUREMENT_AVERAGE_DIFF_TOL)
				  {
					  printf("readjusting to new range baseline\n");
					  prev_range = average_of_prev_measurements;
					  range_rate = (measured_range - prev_range) * LIDAR_DATA_RATE_HZ;
				  } else {
					  std::printf("invalid range rate %f / distance estimation %f \n",range_rate, measured_range);
					  range_rate = prev_range_rate;
					  range = predicted_range;
				  }
			  } else {
				  std::printf("invalid range rate %f \n", range_rate);
				  range_rate = prev_range_rate;
			  }
		  }
	  }
  }
};
