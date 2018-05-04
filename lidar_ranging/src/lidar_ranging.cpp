#include <ros/ros.h>
// ROS message types
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
// ROS image processing libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
// OpenCV specific includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/surface/mls.h>
// c++ std
#include <string>
#include <math.h>

//#define PUBLISH_FILTERED_POINT_CLOUD // if this is enabled, it will also show the cropped/downsampled cloud we generate in rviz
#define VIEW_RAYS // if this is enabled, you can see the rays / distance estimations in the point cloud
#define DEBUG_LOGGING // if this is enabled, distance data and debug data will be published

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 400
#define FRAME_CENTER_X 320;
#define FRAME_CENTER_Y 150;

#define LIDAR_MAX_RANGE 100
#define LIDAR_DATA_RATE_HZ 10
#define LIDAR_DATA_PERIOD_S 0.1
#define MAX_RANGE_RATE 50 // m/s  this is used to filter out new range estimations that would result in unrealistic speeds
#define DEFAULT_RANGE_RATE -1.0 // the assumed initial range rate for a new detection object

#define MAX_NUM_OBJECTS 10 // also change max_num_objects in object_detect.cpp
#define MAX_NO_DETECTION_COUNT 10 // the number of 'no detections' before a tracking object is deactivated
#define MEASUREMENT_LIST_LENGTH 5 // the size of a running list of measurements used for averaging

// measurements can be rejected if they jump significantly from the previous baseline value
// if 5 (rejected) measurements agree with the new measurement within this tol, the new measuremnt will be treated as the new baseline value
#define RANGE_MEASUREMENT_AVERAGE_DIFF_TOL 2

/************** CALIBRATION PARAMETERS ********************/
// The two most useful parameters are C_X_OFFSET and MIN_Z

// this pixel offset corrects the horizontal alignment between the camera frame and lidar data
// -20 to -25 seems to work well
int C_X_OFFSET = -25;

//this is for ground elimination
// up to 0.12 seems to work okay, 0.15 results in accurate points occasionally being rejected in the 10-15 meter range in VehApproachTest5
double MIN_Z = 0.1;

// this can help eliminate erroneous measurements, as most cars / pedestrians / trucks will be under 6m
double MAX_Z = 6;

// camera position relative to the lidar
cv::Point3d camera_position = cv::Point3d(-1.872, 0.0, 0.655);

// the lidar is treated as the origin 0,0,0 (x,y,z)
pcl::PointXYZ origin = pcl::PointXYZ(0,0,0);

// this theta is used to slightly rotate the the ray up or down
// based on my tests, the accuracy is the same without applying this rotation
//#define USE_THETA_Y //enable this to apply the rotation
double theta_y = 0.02;

// if AZIMUTH_TOL_MULTIPLIER is 1, we use an angle equal to the 'angle' of bounding box to search for points
double AZIMUTH_TOL_MULTIPLIER = 0.8;

// shouldn't need to apply downsampling, especially if the sheath is on the lidar (which reduces number of detected points)
// if the performance is very slow, then try applying downsampling
// also downsampling can reduce the range estimation accuracy, especially at further distances > 30 m
bool apply_downsampling = false;

/************** END *********************************************/


//publishers
ros::Publisher pub;
ros::Publisher distancePub;
ros::Publisher range_rate_pub;
ros::Publisher azimuth_pub;
ros::Publisher lateral_distance_pub;
ros::Publisher relative_lane_pub;
ros::Publisher marker_pub;

//camera objects
image_geometry::PinholeCameraModel cam_model_;
//point cloud objects
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
// point cloud filtering obects
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::VoxelGrid<pcl::PointXYZ> sor;

double calculate_average(double[], int);
double estimate_distance_from_histogram(std::vector<double>&);
/************** Section 0: detection object class definition *******************/
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
	double frame_angle_half_width;
  // range estimation variables
  double range;
  double prev_range;
  double range_rate;
  double prev_range_rate;
  double lateral_range;
  int relative_lane;
  unsigned int measurements_index;  
  double measurements[MEASUREMENT_LIST_LENGTH];
  int measurement_count;
  
  DetectionObject()
  {
    id = -1;
    cx = FRAME_CENTER_X;
    cy = FRAME_CENTER_Y;
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
		frame_angle_half_width = 0;
    // range estimation variables
    range = 0;
    prev_range = 0;
    range_rate = 0;
    prev_range_rate = 0;
    lateral_range = 0;
    relative_lane = 0;
    measurements_index = 0;
    measurement_count = 0;
    for (int i = 0; i < MEASUREMENT_LIST_LENGTH; i++) {
      measurements[i] = 0;
    }
  }

  void update_detection_state(bool frame_detected_state)
  {
    frame_detected = frame_detected_state;
	  if(frame_detected)
	  {
		  frame_has_appeared = true;
			no_detection_count = 0;
	  } else if(frame_has_appeared) {
		  no_detection_count++;
		  if(no_detection_count > MAX_NO_DETECTION_COUNT)
		  {
#ifdef DEBUG_LOGGING
			  std::printf("lost the detection\n");
#endif
			  frame_has_appeared = false;
			  no_detection_count = 0;
			  measurement_count = 0;
			  measurements_index = 0;
			  prev_range = 0;
				prev_range_rate = 0;
				range = 0;
        relative_lane = 0;
				for (int i = 0; i < MEASUREMENT_LIST_LENGTH; i++) {
					measurements[i] = 0;
				}
		  }
	  }
  }
  
  void update_detection_frame(int x, int y, int w)
  {
    if(frame_detected)
	  {
      cx = x;
		  cy = y;
			frame_width = w;
    }
  }

  void update_ray(image_geometry::PinholeCameraModel cam_model_, double theta_y)
  {
		int corrected_cx = cx + C_X_OFFSET;
    cv::Point2d frame_center = cv::Point2d(corrected_cx, cy);
	  cv::Point3d camera_ray = cam_model_.projectPixelTo3dRay(frame_center);

		// calculate boundaries representing
		cv::Point2d frame_left_border = cv::Point2d(corrected_cx - frame_width/2, cy);
		cv::Point2d frame_right_border = cv::Point2d(corrected_cx + frame_width/2, cy);
		cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(frame_left_border);
		cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(frame_right_border);
		double left_y = -left_ray.x;
		double right_y = -right_ray.x;
		double dy = std::abs(right_y - left_y)/2 * AZIMUTH_TOL_MULTIPLIER;

	  // convert camera coordinate system to vlp-16 coordinate system convention
	  double ray_world_x = camera_ray.z;
	  double ray_world_z = camera_ray.y;
	  double ray_world_y = -camera_ray.x; // -ve so that the resulting azimuth is not mirrored in the real world

		ray.y = ray_world_y;
		ray.x = ray_world_x;
		ray.z = ray_world_z;
#ifdef USE_THETA_Y
		// apply rotation around y-axis
		ray.x = std::cos(theta_y)*ray_world_x + std::sin(theta_y)*ray_world_z;
	  ray.z = -std::sin(theta_y)*ray_world_x + std::cos(theta_y)*ray_world_z;
#endif
    
    //update azimuth and bearing    
    azimuth = std::atan(ray.y / ray.x);
    elevation = std::atan(ray.z / ray.x);
		frame_angle_half_width = std::abs(std::atan(dy / ray.x));
  }
  
  void update_lateral_range()
  {
    lateral_range = std::sin(azimuth) * range;
  }
  
  void update_relative_lane()
  {
    relative_lane = 2;
    if(lateral_range < -2)
		  relative_lane = 1;
	  else if(lateral_range > 2)
		  relative_lane = 3;
  }
    
  void estimate_range(std::vector<double> distances, int count)
  {
    if(!frame_has_appeared) return;

    double measured_range = prev_range + range_rate * LIDAR_DATA_PERIOD_S;
	  if(count > 0)
	  {
		  measured_range = estimate_distance_from_histogram(distances);
	  } else {
#ifdef DEBUG_LOGGING
		  std::printf("no points detected, estimating based on range rate \n");
#endif
	  }
    range = measured_range;
    range_rate = (measured_range - prev_range) * LIDAR_DATA_RATE_HZ;

    if(no_detection_count <= 2)
    {
      // we add the measurement to the running list of measurements only when a frame is detected, 
      // this is to prevent erroneous estimations added to the list when estimating based on v 
      measurements[measurements_index] = measured_range;
      if(measurement_count < MEASUREMENT_LIST_LENGTH && measured_range != 0)
      {
	      measurement_count++;
      }
      measurements_index = (measurements_index + 1) % MEASUREMENT_LIST_LENGTH;
    } else {
				measurement_count = 0;
				measurements_index = 0;
		    for (int i = 0; i < MEASUREMENT_LIST_LENGTH; i++) {
					measurements[i] = 0;
				}
		}
    filter_range_estimation(measured_range);
    prev_range_rate = range_rate;
    prev_range = range;
    
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
  
  void filter_range_estimation(double measured_range)
  {
	  if(prev_range == 0) {
		  range_rate = DEFAULT_RANGE_RATE;
	  }
	  double predicted_range = prev_range + prev_range_rate * LIDAR_DATA_PERIOD_S;

	  if(range_rate != 0)
	  {
		  if(std::abs(range_rate) > MAX_RANGE_RATE)
		  {
			  if(measurement_count >= MEASUREMENT_LIST_LENGTH)
			  {
          double average_of_prev_measurements = calculate_average(measurements, MEASUREMENT_LIST_LENGTH);
          double measurement_diff_with_average = abs(average_of_prev_measurements - measured_range);
				  if(measurement_diff_with_average < RANGE_MEASUREMENT_AVERAGE_DIFF_TOL)
				  {
#ifdef DEBUG_LOGGING
					  std::printf("readjusting to new range baseline\n");
#endif
					  prev_range = average_of_prev_measurements;
					  range_rate = (measured_range - prev_range) * LIDAR_DATA_RATE_HZ;
				  } else {
#ifdef DEBUG_LOGGING
					  std::printf("invalid range rate %f / distance estimation %f \n",range_rate, measured_range);
#endif
					  range_rate = prev_range_rate;
					  range = predicted_range;
				  }
			  } else {
#ifdef DEBUG_LOGGING
				  std::printf("invalid range rate %f \n", range_rate);
#endif
				  range_rate = prev_range_rate;
			  }
		  }
	  }
  }

  void log_range_data()
  {
    // Log estimated distance
    if(!frame_has_appeared) return;
#ifdef DEBUG_LOGGING
    std::printf("id: %i \t", id);
	  std::printf("range_estimation: %f \t", range);
	  std::printf("range rate: %f \t", range_rate);
	  std::printf("camera x,y : (%i , %i) \t", cx, cy);
    std::printf("azimuth: %f \t", azimuth);
    std::printf("lane: %i \t", relative_lane);
	  std::printf("frame detected : %i \n", frame_detected);
#endif
  }
};

/************** END *******************/
// global variable of list of tracked objects
std::vector<DetectionObject> detection_objects;

/************** Section 1: Functions for processing the point cloud ************/
void apply_passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 10);
	pass.filter(*cloud_filtered);
	
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (0, 100);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-50, 50);
	pass.filter(*cloud_filtered);
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*cloud_filtered);
}
/**************************** END *******************************************/

/************** Section 2: Functions for distance estimation ************/
float calculate_distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	float dz = p1.z - p2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void estimate_ranges_for_all_detected_objects(cv::Point3d camera_position, pcl::PointXYZ origin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	std::vector<std::vector<double> > distances(MAX_NUM_OBJECTS, std::vector<double>(0));
	std::vector<std::vector<double> >distances_2(MAX_NUM_OBJECTS, std::vector<double>(0));
	std::vector<int> count(MAX_NUM_OBJECTS, 0);
	std::vector<int> count_2(MAX_NUM_OBJECTS, 0);
	int num_points = cloud_filtered->points.size();

  // iterate through point cloud
	for(int i = 0; i < num_points; i++)
  {
		float x = cloud_filtered->points[i].x;
		float y = cloud_filtered->points[i].y;
		float z = cloud_filtered->points[i].z;
		
		double dx = x - camera_position.x;
		double dy = y - camera_position.y;
		double dz = z - camera_position.z;

		double point_elevation = dz / dx;
		double point_bearing = dy / dx;
		
    double distance = calculate_distance(cloud_filtered->points[i], origin);
		double elevation_tolerance = 0.01;

    // iterate through detection objects
    for (int j = 0; j < MAX_NUM_OBJECTS; j++)
    {
			if(!detection_objects[j].frame_has_appeared) continue;
      double ray_elevation = detection_objects[j].elevation;
      double ray_azimuth = detection_objects[j].azimuth;
      double elevation_diff = std::abs(ray_elevation - point_elevation);
		  double azimuth_diff = std::abs(ray_azimuth - point_bearing);
			double azimuth_tolerance = detection_objects[j].frame_angle_half_width;
      if(azimuth_diff <= azimuth_tolerance && distance > 0.01 && z > MIN_Z && z < MAX_Z)
		  {
			  distances[j].push_back(distance);
			  count[j]++;
		  } else if (azimuth_diff <= 2*azimuth_tolerance && distance > 0.01 && z > MIN_Z && z < MAX_Z) {
			  distances_2[j].push_back(distance);
			  count_2[j]++;
		  }
    }
  }
  // iterate through detection objects, getting distance estimate from histogram
  for (int j = 0; j < MAX_NUM_OBJECTS; j++)
  {
    if(!detection_objects[j].frame_has_appeared) continue;
    double range_estimation = detection_objects[j].prev_range;
	  // generate sorted list of points
	  if(count[j] > 0) {
		  std::sort (distances[j].begin(), distances[j].end());
      detection_objects[j].estimate_range(distances[j], count[j]);
	  } else if (count_2[j] > 0) {
		  std::sort (distances_2[j].begin(), distances_2[j].end());
		  detection_objects[j].estimate_range(distances_2[j], count_2[j]);
#ifdef DEBUG_LOGGING
		  std::printf("loosened tolerance to find enough points \n");
#endif
	  } else {
      detection_objects[j].estimate_range(distances[j], 0);
    }
    detection_objects[j].log_range_data();
  }
}

double estimate_distance_from_histogram(std::vector<double>& distances)
{
	double min_distance = 0;
	double max_distance = 100;
	double bin_width = 1;
	int num_bins = (max_distance - min_distance) / bin_width + 1;	
	std::vector<int> bin_count(num_bins);
	std::vector<double> bin_averages(num_bins);

	for (int i = 0; i < distances.size(); i++)
	{
		int bin_value = std::floor(distances[i]);
		int bin_index = std::ceil((bin_value - min_distance) / bin_width);
		bin_count[bin_index]++;
		bin_averages[bin_index]+=distances[i];
	}
	for(int i = 0; i < num_bins; i++)
	{
		if(bin_count[i] > 0)
		{
			bin_averages[i] /= double(bin_count[i]);
		}
	}
	int winning_index = 0;
	int winning_count = -1;
	for(int i = 0; i < num_bins; i++)
	{
		if(bin_count[i] > winning_count)
		{
			winning_index = i;
			winning_count = bin_count[i];
		}
	}
	double winning_distance = bin_averages[winning_index];
	return winning_distance;
}

geometry_msgs::Point spherical_to_cartesian(double r, double elevation, double azimuth)
{
		geometry_msgs::Point point;
		point.x = r * std::cos(azimuth) * std::cos(elevation);
		point.y = r * std::cos(elevation) * std::sin(azimuth);
		point.z = r * std::sin(elevation);		
		return point;
}

double calculate_ray_length(cv::Point3d ray)
{
	return std::sqrt(ray.x*ray.x + ray.y*ray.y + ray.z*ray.z);
}

cv::Point3d calculate_endpoint(cv::Point3d ray, cv::Point3d start_point, double r)
{
	cv::Point3d end_point;
	double myx = ray.y / ray.x;
  double mzx = ray.z / ray.x;

	double ray_length = calculate_ray_length(ray);
	double cos_azimuth = ray.x / ray_length;

	float dx = cos_azimuth * r;
  float dy = dx*myx;
  float dz = dx*mzx;
  end_point.x = start_point.x + dx;
  end_point.y = start_point.y + dy;
  end_point.z = start_point.z + dz;
	return end_point;
}

geometry_msgs::Point convert_cv_point_to_geometry_msgs(cv::Point3d point)
{
	geometry_msgs::Point point_msg;
	point_msg.x = point.x;
	point_msg.y = point.y;
	point_msg.z = point.z;
	return point_msg;
}

double calculate_average(double measurements[], int count)
{
	double average = 0;
	for (int i = 0; i < count; i++) {
		average += measurements[i];
	}
	return average/double(count);
}

/**************************** END *******************************************/


/************** Section 3: Functions for visualizing the distance estimation ************/
void draw_line(cv::Point3d start_point, cv::Point3d end_point, ros::Publisher marker_pub, double line_r, double line_g, double line_b, std::string name, int obj_id)
{
		std::string frame_id = "velodyne";
		std::string ns = name + boost::lexical_cast<std::string>(obj_id);
		visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = frame_id;
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = ns;
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

	  // POINTS markers use x and y scale for width/height respectively
	  points.scale.x = 0.2;
	  points.scale.y = 0.2;

	  // LINE_STRIP markers use only the x component of scale, for the line width
	  line_strip.scale.x = 0.1;

	  // Points are green
	  points.color.g = 1.0f;
	  points.color.a = 1.0;

	  // Line strip is blue
		line_strip.color.r = line_r;
		line_strip.color.g = line_g;
		line_strip.color.b = line_b;
	  line_strip.color.a = 1.0;

		// Populate point list and line strip
		geometry_msgs::Point start = convert_cv_point_to_geometry_msgs(start_point);
		geometry_msgs::Point end = convert_cv_point_to_geometry_msgs(end_point);
		points.points.push_back(start);
    line_strip.points.push_back(start);
    points.points.push_back(end);
    line_strip.points.push_back(end);

		// Publish the point and line markers
  	marker_pub.publish(points);
  	marker_pub.publish(line_strip);
}

double draw_search_boundaries(cv::Point3d ray, cv::Point3d start_point, double r, double angle_tolerance, ros::Publisher marker_pub, int obj_id)
{
	double ray_length = calculate_ray_length(ray);
	double myx = ray.y / ray.x;
	double theta_center = std::atan(myx);
	double theta_left = theta_center - angle_tolerance;
	double theta_right = theta_center + angle_tolerance;

	cv::Point3d ray_left;
	ray_left.x = ray_length * cos(theta_left);
	ray_left.y = ray_length * sin(theta_left);

	cv::Point3d ray_right;
	ray_right.x = ray_length * cos(theta_right);
	ray_right.y = ray_length * sin(theta_right);

	cv::Point3d left_end_point = calculate_endpoint(ray_left,start_point, r);
	draw_line(start_point, left_end_point, marker_pub, 1.0, 0.0, 0.0, "offset_left_ray", obj_id);

	cv::Point3d right_end_point = calculate_endpoint(ray_right,start_point, r);
	draw_line(start_point, right_end_point, marker_pub, 1.0, 0.0, 0.0, "offset_right_ray", obj_id);
}

void visualize_ray(DetectionObject detection_object, ros::Publisher marker_pub)
{
  // modified line
	cv::Point3d projected_ray_xy_plane = cv::Point3d(detection_object.ray.x, detection_object.ray.y, 0);
	// Draw projected ray
	cv::Point3d lidar_position = cv::Point3d(0, 0, 0);
	cv::Point3d end_point_50m = calculate_endpoint(projected_ray_xy_plane,lidar_position, 50);

	int obj_id = detection_object.id;
	//draw_line(lidar_position, end_point_50m, marker_pub, 0.0, 0.0, 1.0, "points_and_lines_50m");
	cv::Point3d end_point = calculate_endpoint(projected_ray_xy_plane,lidar_position, detection_object.range);
	draw_line(lidar_position, end_point, marker_pub, 0.0, 1.0, 0.0, "points_and_lines", obj_id);
	draw_search_boundaries(projected_ray_xy_plane, lidar_position, 50, detection_object.frame_angle_half_width, marker_pub, obj_id);
}

void view_all_rays(ros::Publisher marker_pub)
{
	for (int i = 0; i < MAX_NUM_OBJECTS; i++)
	{
		if(detection_objects[i].frame_has_appeared)
		{
		  visualize_ray(detection_objects[i], marker_pub);
		}
	}
}
/**************************** END *******************************************/


/************** Section 4: data logging functions ********************************/

void publish_can_data()
{
  std_msgs::MultiArrayLayout layout = std_msgs::MultiArrayLayout();
  layout.dim.push_back(std_msgs::MultiArrayDimension());
  layout.dim[0].size = MAX_NUM_OBJECTS;
  layout.dim[0].stride = 1;
  layout.dim[0].label = "length";

  std_msgs::Float32MultiArray dist_msg;
  dist_msg.layout = layout;
  std_msgs::Float32MultiArray range_rate_msg;
  range_rate_msg.layout = layout;
  std_msgs::Float32MultiArray azimuth_msg;
  azimuth_msg.layout = layout;
  std_msgs::Float32MultiArray lateral_msg;
  lateral_msg.layout = layout;
  std_msgs::Int32MultiArray lane_msg;
  lane_msg.layout = layout;
  
  for (int i = 0; i < MAX_NUM_OBJECTS; i++)
  {
    dist_msg.data.push_back(detection_objects[i].range);
    range_rate_msg.data.push_back(detection_objects[i].range_rate);
    azimuth_msg.data.push_back(detection_objects[i].azimuth);
    lateral_msg.data.push_back(detection_objects[i].lateral_range);
    lane_msg.data.push_back(detection_objects[i].relative_lane);
  }
  
  // Long range
	distancePub.publish(dist_msg);
  // Range rate
  range_rate_pub.publish(range_rate_msg);
	// Azimuth
	azimuth_pub.publish(azimuth_msg);
	// lateral range
	lateral_distance_pub.publish(lateral_msg);
  // relative lane
	relative_lane_pub.publish(lane_msg);
  
}

/**************************** END *******************************************/

/************** Section 5: callback functions ********************************/
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(!cam_model_.initialized()) return;
	
  // 1. Process Point Cloud
	// Convert message to pcl::PointCloud type
	pcl::fromROSMsg (*input, *cloud);
	// Apply passthrough filter
	apply_passthrough_filter(cloud, cloud_filtered);
	// Downsampling
	if(apply_downsampling)
	{
		downsample(cloud_filtered, cloud_filtered);
	}
#ifdef PUBLISH_FILTERED_POINT_CLOUD
	// Publish point cloud
  sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish (output);
#endif

  // 2. Get range estimations for all detection objects
  estimate_ranges_for_all_detected_objects(camera_position, origin, cloud_filtered);

	// 3. Visualize range estimation
#ifdef VIEW_RAYS
  view_all_rays(marker_pub);
#endif

	// 4. Publish data for CAN bus
  publish_can_data();
}

void frame_cb(const geometry_msgs::PoseArray pose_msg)
{
	for (int i=0; (i < pose_msg.poses.size()) && i < (MAX_NUM_OBJECTS); i++)
	{
		int x = pose_msg.poses[i].position.x;
		int y = pose_msg.poses[i].position.y;
		int w = pose_msg.poses[i].position.z;
		detection_objects[i].update_detection_frame(x,y,w);
		detection_objects[i].update_ray(cam_model_,theta_y);
	}
}

void frame_detected_cb(const std_msgs::Int8MultiArray& frame_detected_msg)
{
	for (int i = 0; (i < frame_detected_msg.data.size()) && i < (MAX_NUM_OBJECTS); i++)
	{
		detection_objects[i].update_detection_state(frame_detected_msg.data[i]);
	}
}

void camera_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Initialize camera model
  if(!cam_model_.initialized())
  {
    cam_model_.fromCameraInfo(*info_msg);
		std::printf("camera model initialized\n");
  }
}

/**************************** END *******************************************/

/**************************** Main function *******************************************/
int main (int argc, char** argv)
{
	std::printf("running lidar ranging\n");
	if(argc > 1) {
		AZIMUTH_TOL_MULTIPLIER = std::atof(argv[1]);
		std::printf("bearing tol multiplier: %f\n", AZIMUTH_TOL_MULTIPLIER);
	} if (argc > 2) {
		MIN_Z = std::atof(argv[2]);
		std::printf("min z: %f\n", MIN_Z);
	} if (argc > 3) {
		C_X_OFFSET = std::atof(argv[3]);
		std::printf("cx offset: %i\n", C_X_OFFSET);
	}
  // Initialize ROS
  ros::init (argc, argv, "lidar_ranging");
  ros::NodeHandle nh;

  std::printf("initializing detection object \n");

  detection_objects.reserve(MAX_NUM_OBJECTS);
	for(int i = 0; i < MAX_NUM_OBJECTS;i ++)
	{
    DetectionObject detectionObject;
    detectionObject.id = i;
		detection_objects[i] = detectionObject;
	}

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar/cropped_cloud", 1);

	// Create a ROS subscriber for the camera info
	ros::Subscriber camera_info_sub = nh.subscribe("/videofile_test/camera_info", 1, camera_cb);

	// Create a ROS::ImageTransport publisher for the output point cloud
	image_transport::ImageTransport it(nh);

	// Create ROS subscriber for frame_center_sub
	ros::Subscriber frame_center_sub = nh.subscribe("/darknet_ros/frame_center",1,frame_cb);
	ros::Subscriber frame_detected_sub = nh.subscribe("/darknet_ros/frame_detected",1,frame_detected_cb);

	distancePub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_ranging/distance",1);
  range_rate_pub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_ranging/range_rate",1);
	azimuth_pub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_ranging/azimuth",1);
	lateral_distance_pub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_ranging/lateral_distance",1);
	relative_lane_pub = nh.advertise<std_msgs::Int32MultiArray>("/lidar_ranging/relative_lane",1);

	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	// Spin
  ros::spin();
}
/**************************** END *******************************************/
