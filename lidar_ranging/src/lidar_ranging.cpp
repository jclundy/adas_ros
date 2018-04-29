#include <ros/ros.h>
// ROS message types
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
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
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/surface/mls.h>
// c++ std
#include <math.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 400
#define LIDAR_MAX_RANGE 100

#define CENTER_X 320
#define CENTER_U 200

int frame_center_X = 320;
int frame_center_Y = 150;

bool frame_detected = false;
int no_detection_count = 0;
double previous_distance = 0;
bool frame_has_appeared = false;

double old_range = 0;
double range_rate = 0;
double prev_range_rate = 0;
double prev_range = 100;

ros::Time prev_time;
ros::Time current_time;
ros::Time start_time;
int range_measurements = 0;

cv::Point3d camera_position = cv::Point3d(-1.872, 0.0, 0.655);
pcl::PointXYZ origin = pcl::PointXYZ(0,0,0);
double theta_y = 0.02;

//publishers
ros::Publisher pub;
ros::Publisher pub_2;

ros::Publisher distancePub;
ros::Publisher azimuth_pub;
ros::Publisher lateral_distance_pub;
ros::Publisher relative_lane_pub;

ros::Publisher pointPublisher;
ros::Publisher marker_pub;

//camera objects
image_transport::Publisher range_image_pub;
image_geometry::PinholeCameraModel cam_model_;
sensor_msgs::CameraInfo camera_info_msg;

//point cloud objects
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// point cloud filtering obects
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::VoxelGrid<pcl::PointXYZ> sor;

// range pixels 
cv::Mat range_pixels; // (2d array of ranges)
cv::Mat range_gray_pixels;// (2d array, with ranges mapped between 0 - 255, greyscale - for visualization purposes)
cv::Mat range_rgb_pixels;// (3d array, with range from 0-255 mapped to rgb color - for visualization purposes)

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

float calculate_distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	float dz = p1.z - p2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int get_list_of_distances(std::vector<double> &distances_out, std::vector<double>& latitudes, 
													cv::Point3d camera_position, pcl::PointXYZ origin, double ray_elevation, double ray_bearing,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	std::vector<double> distances_1deg(0);
	int count = 0;
	int num_points = cloud_filtered->points.size();
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
		
		double elevation_diff = std::abs(ray_elevation - point_elevation);
		double bearing_diff = std::abs(ray_bearing - point_bearing);
		
		double elevation_tolerance = 0.01;
		double bearing_tolerance = 0.0174533;
		if(distance <= 20)
		{
			bearing_tolerance = 0.034;
		}
		if(distance <=10)
		{
			bearing_tolerance = 0.0873;
			elevation_tolerance = 0.05;
		}
		//if(elevation_diff <= elevation_tolerance && bearing_diff <= bearing_tolerance && distance > 0.01)

		if(bearing_diff <= bearing_tolerance && distance > 0.01 && z > 0)
		{
			distances_1deg.push_back(distance);
			latitudes.push_back(y);
			count++;
		}
  }
	// generate sorted list of points
	std::sort (distances_1deg.begin(), distances_1deg.end());
	std::sort (latitudes.begin(), latitudes.end());
	distances_out = distances_1deg;
	// print distances
	/*std::printf("sorted distance list \n");
	std::printf("%i points\n", count);
	for(int i = 0; i < count; i++)
	{
		std::printf("%f,", distances[i]);
	}*/
	return count;
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
	// print histogram
	/*std::printf("Histogram \n");
	int winning_bin_index = 0;
	for(int i = 0; i < num_bins; i++)
	{
		if(bin_count[i] > 0)
		{
			double distance = i * bin_width + min_distance;
			std::printf("distance: %f, count %i \n", distance, bin_count[i]);
		}
	}*/
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

cv::Point3d calculate_ray(int frame_center_X, int frame_center_Y, image_geometry::PinholeCameraModel cam_model_, double theta_y)
{
	// get ray from pixel using pinhole camera model
	cv::Point2d frame_center = cv::Point2d(frame_center_X, frame_center_Y);
	cv::Point3d ray = cam_model_.projectPixelTo3dRay(frame_center);
	cv::Point3d ray_rotated;
	
	// convert camera coordinate system to vlp-16 coordinate system convention
	double ray_world_x = ray.z;
	double ray_world_z = ray.y;
	double ray_world_y = ray.x;

	// apply rotation
	ray_rotated.x = std::cos(theta_y)*ray_world_x + std::sin(theta_y)*ray_world_z;
	ray_rotated.z = -std::sin(theta_y)*ray_world_x + std::cos(theta_y)*ray_world_z;
	ray_rotated.y = ray_world_y;
	return ray_rotated;
}

void draw_line(cv::Point3d ray, double r, cv::Point3d camera_position, ros::Publisher marker_pub)
{
		std::string frame_id = "velodyne";
		std::string ns = "points_and_lines";
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
	  line_strip.color.b = 1.0;
	  line_strip.color.a = 1.0;

	  double myx = ray.y / ray.x;
	  double mzx = ray.z / ray.x;
		double num_intervals = 2;

	  // Create the vertices for the points and lines
	  for (uint32_t i = 0; i < num_intervals; ++i)
	  {
	    float dx = i*r/num_intervals;
	    float dy = dx*myx;
	    float dz = dx*mzx;

	    geometry_msgs::Point p;
	    p.x = camera_position.x + dx;
	    p.y = camera_position.y + dy;
	    p.z = camera_position.z + dz;

	    points.points.push_back(p);
	    line_strip.points.push_back(p);
	  } 
  	marker_pub.publish(points);
  	marker_pub.publish(line_strip);
}

double estimate_distance(cv::Point3d ray, 
													double prev_distance, 
													cv::Point3d camera_position, 
													pcl::PointXYZ origin,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	double lidar_elevation = ray.z / ray.x;
	double lidar_bearing = ray.y / ray.x;
		
  // iterate through point cloud by index
	std::vector<double> distances(0);
	std::vector<double> latitudes(0);
	
	// project ray onto xy plane, by setting camera_position = origin
	// 
	cv::Point3d lidar_position = cv::Point3d(0.0, 0.0, 0.0);
	int count = get_list_of_distances(distances, latitudes, lidar_position, origin, lidar_elevation, lidar_bearing, cloud_filtered);
	double distance = prev_distance;
	if(count > 0) 
	{
		distance = estimate_distance_from_histogram(distances);
	}
	return distance;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Convert message to pcl::PointCloud type
	pcl::fromROSMsg (*input, *cloud);
	// Apply passthrough filter
	apply_passthrough_filter(cloud, cloud_filtered);
	// Downsampling
	//downsample(cloud_filtered, cloud_filtered);
  	
	if(!cam_model_.initialized()) return;
	if(!frame_has_appeared) return;
	
	cv::Point3d ray = calculate_ray(frame_center_X, frame_center_Y, cam_model_,theta_y);	
  double winning_distance = estimate_distance(ray, 
													prev_range, 
													camera_position, 
													origin, 
													cloud_filtered);
	prev_range = winning_distance;
	double lidar_elevation = ray.z / ray.x;
	double lidar_bearing = ray.y / ray.x;
	double lateral_range = std::sin(lidar_bearing) * winning_distance;
	geometry_msgs::PointStamped estimated_point;
	estimated_point.point = spherical_to_cartesian(winning_distance, lidar_elevation, lidar_bearing);	

	// modified line
	cv::Point3d projected_ray_xy_plane = cv::Point3d(ray.x, ray.y, 0);
	// Draw projected ray
	cv::Point3d lidar_position = cv::Point3d(0, 0, 0);
	draw_line(projected_ray_xy_plane, 100, lidar_position, marker_pub);	

	// Publish estimated point
	pointPublisher.publish(estimated_point);
	// Publish point cloud
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish (output);

	// Log estimated distance
	std::printf("winning_distance: %f \t", winning_distance);
	std::printf("camera x,y : (%i , %i) \n", frame_center_X, frame_center_Y);

	// Publish info for CAN bus
	// Long range
	std_msgs::Float32 dist_msg;
	dist_msg.data = winning_distance;
	distancePub.publish(dist_msg);
	// Azimuth
	std_msgs::Float32 azimuth_msg;
	azimuth_msg.data = lidar_bearing;
	azimuth_pub.publish(azimuth_msg);
	// lateral range
	std_msgs::Float32 lateral_msg;
	lateral_msg.data = lateral_range;
	lateral_distance_pub.publish(lateral_msg);

	// hacky lane detection
	std_msgs::Int32 lane_msg;
	lane_msg.data = 2;
	if(lateral_range < -2)
		lane_msg.data = 1;
	else if(lateral_range > 2)
		lane_msg.data = 3;
	relative_lane_pub.publish(lane_msg);
}

void frame_cb(const geometry_msgs::Pose2D& pose_msg)
{
	//frame_detected = true;
	if(frame_detected)
	{
		frame_center_X = pose_msg.x;
		frame_center_Y = pose_msg.y;
	}
}

void frame_detected_cb(const std_msgs::Bool& frame_detected_msg)
{
	if(frame_detected_msg.data)
	{
		frame_detected = true;
		frame_has_appeared = true;
	} else {
		no_detection_count++;
		if(no_detection_count > 40)
		{
			frame_detected = false;
			no_detection_count = 0;
		}
	}
}

void camera_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Initialize camera model
  if(!cam_model_.initialized())
  {
    cam_model_.fromCameraInfo(*info_msg);
  }
}

int main (int argc, char** argv)
{
	if(argc > 1) {
		theta_y = std::atof(argv[1]);
	}
  // Initialize ROS
  ros::init (argc, argv, "lidar_ranging");
  ros::NodeHandle nh;
	
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
	
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar/cropped_cloud", 1);

	// Create a ROS subscriber for the camera info
	ros::Subscriber camera_info_sub = nh.subscribe("/videofile_test/camera_info", 1, camera_cb);
	
	// Create a ROS::ImageTransport publisher for the output point cloud
	image_transport::ImageTransport it(nh);
	range_image_pub = it.advertise("lidar/range_image",1);
	
	// Create ROS subscriber for frame_center_sub
	ros::Subscriber frame_center_sub = nh.subscribe("/darknet_ros/frame_center",1,frame_cb);
	ros::Subscriber frame_detected_sub = nh.subscribe("/darknet_ros/frame_detected",1,frame_detected_cb);
  
	distancePub = nh.advertise<std_msgs::Float32>("/darknet_ros/distance",1);
	azimuth_pub = nh.advertise<std_msgs::Float32>("/lidar_ranging/azimuth",1);
	lateral_distance_pub = nh.advertise<std_msgs::Float32>("/lidar_ranging/lateral_distance",1);
	relative_lane_pub = nh.advertise<std_msgs::Int32>("/lidar_ranging/relative_lane",1);

	pointPublisher = nh.advertise<geometry_msgs::PointStamped>("/lidar_ranging/range_point",1);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	// Spin
  ros::spin ();
}
