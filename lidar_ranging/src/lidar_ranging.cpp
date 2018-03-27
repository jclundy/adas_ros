#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
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

double prev_range = 100;

//publishers
ros::Publisher pub;
ros::Publisher pub_2;
ros::Publisher distancePub;
ros::Publisher pointPublisher;

//camera objects
image_transport::Publisher range_image_pub;
image_geometry::PinholeCameraModel cam_model_;
sensor_msgs::CameraInfo camera_info_msg;

//point cloud objects
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

// point cloud filtering obects
pcl::PassThrough<pcl::PointXYZRGB> pass;
pcl::VoxelGrid<pcl::PointXYZRGB> sor;
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity(); // Matrix transform from camera pose -> lidar pose

// range pixels 
cv::Mat range_pixels; // (2d array of ranges)
cv::Mat range_gray_pixels;// (2d array, with ranges mapped between 0 - 255, greyscale - for visualization purposes)
cv::Mat range_rgb_pixels;// (3d array, with range from 0-255 mapped to rgb color - for visualization purposes)

void apply_passthrough_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
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

void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*cloud_filtered);
}

float calculate_distance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	float dz = p1.z - p2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int get_list_of_distances(std::vector<double> &distances_out, std::vector<double>& latitudes, 
													pcl::PointXYZRGB origin, double lidar_elevation, double lidar_bearing, 
													pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
	std::vector<double> distances_1deg(0);
	std::vector<double> distances_2deg(0);
	std::vector<double> distances_5deg(0);
	int count = 0;
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
		
		double tolerance = 0.0174533;
		if(distance <= 20)
		{
			tolerance = 0.034;
		}
		if(distance <=10)
		{
			tolerance = 0.0873;
		}
		if(elevation_diff <= tolerance && bearing_diff <= tolerance && distance > 0.01)
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
	double lidar_bearing = lidar_y / lidar_x;
	pcl::PointXYZRGB origin(0,0,0);
		
  // iterate through point cloud by index
	std::vector<double> distances(0);
	std::vector<double> latitudes(0);

	int count = get_list_of_distances(distances, latitudes, origin,lidar_elevation, lidar_bearing, cloud_filtered);
	
	double winning_distance = prev_range;
	if(count > 0) 
	{
		winning_distance = estimate_distance_from_histogram(distances);
		prev_range = winning_distance;
	}
	if(frame_has_appeared)
	{
		std::printf("winning_distance: %f \t", winning_distance);
		std::printf("count: %i \t", count);
		std::printf("camera x,y : (%i , %i) \n", frame_center_X, frame_center_Y);
		std_msgs::Float32 dist_msg;
		dist_msg.data = winning_distance;
		distancePub.publish(dist_msg);
	
		geometry_msgs::PointStamped estimated_point;
		estimated_point.point.x = winning_distance;
		estimated_point.point.y = lidar_bearing * winning_distance;
		estimated_point.point.z = lidar_elevation * winning_distance;		
		pointPublisher.publish(estimated_point);
	}
	/*if(!std::isnan(average_distance) && !std::isnan(old_range))
	{
		range_rate = (old_range - average_distance) * 10 + prev_range_rate / 2;
	} else {
		range_rate = std::nan("100");
	}	*/
	
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
  // Initialize ROS
  ros::init (argc, argv, "lidar_ranging");
  ros::NodeHandle nh;
  
	// Define a translation matrix
	transform_2.translation() << 1.872, 0.0, -0.655;

	// The same rotation matrix as before; theta radians arround Z axis
	//transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	
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
	pointPublisher = nh.advertise<geometry_msgs::PointStamped>("/lidar_ranging/range_point",1); 	
	// Spin
  ros::spin ();
}
