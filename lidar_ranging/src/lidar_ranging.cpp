#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
// ROS specific includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
// OpenCV specific includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
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

double prev_range = 0;
ros::Time prev_time;
ros::Time current_time;
ros::Time start_time;
int range_measurements = 0;

int count_for_range_rate = 0;
double current_range_ = 0;
double winning_distance = 0;
double lateral_range = 0;
double lidar_bearing = 0;

//publishers
ros::Publisher pub;
ros::Publisher pub_2;

ros::Publisher distancePub;
ros::Publisher azimuth_pub;
ros::Publisher lateral_distance_pub;
ros::Publisher relative_lane_pub;

ros::Publisher pointPublisher;

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
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity(); // Matrix transform from camera pose -> lidar pose

// range pixels 
cv::Mat range_pixels; // (2d array of ranges)
cv::Mat range_gray_pixels;// (2d array, with ranges mapped between 0 - 255, greyscale - for visualization purposes)
cv::Mat range_rgb_pixels;// (3d array, with range from 0-255 mapped to rgb color - for visualization purposes)

void apply_passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
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

void copy(std::vector<double> &out, std::vector<double> &in)
{
	for(int i = 0; i < in.size(); i++)
	{
			out.push_back(in[i]);
	}
}


int get_list_of_distances(std::vector<double> &distances_out, std::vector<double>& latitudes, 
													pcl::PointXYZ origin, double lidar_elevation, double lidar_bearing, 
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	std::vector<double> distances_1deg(0);
	distances_1deg.reserve(200);
	std::vector<double> distances_2deg(0);
	distances_2deg.reserve(200);
	std::vector<double> distances_5deg(0);
	distances_5deg.reserve(200);
	
	int count_1deg = 0;
	int count_2deg = 0;
	int count_5deg = 0;
	int num_points = cloud_filtered->points.size();
	for(int i = 0; i < num_points; i++)
  {	
    
		float x = cloud_filtered->points[i].x;
		float y = cloud_filtered->points[i].y;
		float z = cloud_filtered->points[i].z;
		double point_elevation = z / x;
		double point_bearing = y / x;
		
    double distance = calculate_distance(cloud_filtered->points[i], origin);
		
		double elevation_diff = std::abs(lidar_elevation - point_elevation);
		double bearing_diff = std::abs(lidar_bearing - point_bearing);
		
		double tolerance_1deg = 0.0174533;
		double tolerance_2deg = 0.035;
		double tolerance_5deg = 0.0873;

		if(bearing_diff <= tolerance_1deg && distance > 0.01)
		{
			distances_1deg.push_back(distance);
			latitudes.push_back(y);
			count_1deg++;
		} else if(bearing_diff <= tolerance_2deg && distance > 0.01) {
			distances_2deg.push_back(distance);
			latitudes.push_back(y);
			count_2deg++;
		} else if(bearing_diff <= tolerance_5deg && distance > 0.01) {
			distances_5deg.push_back(distance);
			latitudes.push_back(y);
			count_5deg++;
		}
  }
	// generate sorted list of points
	int count = 0;
	if(count_1deg > 0) {
		copy(distances_out, distances_1deg);
		count = count_1deg;
		if(frame_has_appeared) std::printf("1 deg list");
	} else if (count_2deg > 0) {
		copy(distances_out, distances_2deg);
		count = count_2deg;
		if(frame_has_appeared) std::printf("2 deg list");
	} else {
		copy(distances_out, distances_5deg);
		count = count_5deg;
		if(frame_has_appeared)std::printf("5 deg list");
	}

	/*if(count > 0)
	{
		std::sort (distances_out.begin(), distances_out.end());
		std::sort (latitudes.begin(), latitudes.end());
	}*/
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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Convert message to pcl::PointCloud type
  pcl::fromROSMsg (*input, *cloud);
	
  // Apply passthrough filter
	apply_passthrough_filter(cloud, cloud_filtered);
	
  // Downsampling
  //downsample(cloud_filtered, cloud_filtered);
  
  // Matrix transformation
  pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_2);  
  
	// Publish point cloud
  sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
  pub.publish (output);
	
	if(!cam_model_.initialized()) return;
	
	cv::Point2d frame_center = cv::Point2d(frame_center_X, frame_center_Y);
	cv::Point3d ray = cam_model_.projectPixelTo3dRay(frame_center);
	double ray_x = ray.x;
	double ray_y = ray.y;
	double ray_z = ray.z;

	double lidar_x = ray_z;
	double lidar_z = ray.y;
	double lidar_y = ray.x;

	double lidar_elevation = lidar_z / lidar_x;
	lidar_bearing = lidar_y / lidar_x;
	pcl::PointXYZ origin(0,0,0);
		
	std::vector<double> distances(0);
	std::vector<double> latitudes(0);

	
	if(frame_has_appeared)
	{
		int count = get_list_of_distances(distances, latitudes, origin,lidar_elevation, lidar_bearing, cloud_filtered);
	
		winning_distance = prev_range;
		if(count > 0) 
		{
			winning_distance = estimate_distance_from_histogram(distances);
			prev_range = winning_distance;
		}		
		
		lateral_range = lidar_bearing * winning_distance;
		geometry_msgs::PointStamped estimated_point;
		estimated_point.point.x = winning_distance;
		estimated_point.point.y = lidar_bearing * winning_distance;
		estimated_point.point.z = lidar_elevation * winning_distance;		
		pointPublisher.publish(estimated_point);
		
		//std::printf("winning_distance: %f \t", winning_distance);
		//std::printf("count: %i \t", count);
		//std::printf("camera x,y : (%i , %i) \n", frame_center_X, frame_center_Y);
		std_msgs::Float32 dist_msg;
		dist_msg.data = winning_distance;
		distancePub.publish(dist_msg);
		
		std_msgs::Float32 azimuth_msg;
		azimuth_msg.data = lidar_bearing;
		azimuth_pub.publish(azimuth_msg);
		
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
}

void frame_cb(const geometry_msgs::Pose2D& pose_msg)
{
	//frame_detected = true;
	if(frame_has_appeared)
	{
			range_measurements ++;
			if(range_measurements > 100) range_measurements = 2;
	}
	if(range_measurements > 1)
	{
		current_time = ros::Time::now();
		double time_diff = 0.05;
		
		ROS_INFO("Long range, Long range rate, azimuth, Lateral Range, : %f, %f, %f, %f \n", winning_distance, range_rate, lidar_bearing, lateral_range);
		prev_range = winning_distance;
		prev_time = current_time;
	}

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

void range_rate_cb(const std_msgs::Float32& info_msg)
{
	range_rate = info_msg.data;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_ranging");
  ros::NodeHandle nh;
  
	// Define a translation matrix
	transform_2.translation() << 1.872, 0.0, -0.655;
	
	//double theta  = 0.0174533;
	// The same rotation matrix as before; theta radians arround Z axis
	//transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
	
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
	
	ros::Subscriber range_rate_sub = nh.subscribe("/lidar_ranging/range_rate",1,range_rate_cb);  

	pointPublisher = nh.advertise<geometry_msgs::PointStamped>("/lidar_ranging/range_point",1);

	distancePub = nh.advertise<std_msgs::Float32>("/lidar_ranging/distance",1);
	azimuth_pub = nh.advertise<std_msgs::Float32>("/lidar_ranging/azimuth",1);
	lateral_distance_pub = nh.advertise<std_msgs::Float32>("/lidar_ranging/lateral_distance",1);
	relative_lane_pub = nh.advertise<std_msgs::Int32>("/lidar_ranging/relative_lane",1);
	// Spin
  ros::spin();
}
