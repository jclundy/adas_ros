#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
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

double old_range = 0;
double range_rate = 0;
double prev_range_rate = 0;


//publishers
ros::Publisher pub;
ros::Publisher pub_2;
ros::Publisher distancePub;

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

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-10, 10);
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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Convert message to pcl::PointCloud type
  pcl::fromROSMsg (*input, *cloud);
	
  // Apply passthrough filter
	apply_passthrough_filter(cloud, cloud_filtered);
	
  // Downsampling
  downsample(cloud_filtered, cloud_filtered);
  
  // Matrix transformation
  pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_2);  
  
	// Publish point cloud
  sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
  pub.publish (output);

  if(cam_model_.initialized())
  {
		int num_points = cloud_filtered->points.size();
		//ROS_INFO("Number of points in cloud %i", num_points);
		
		cv::Point2d frame_center = cv::Point2d(frame_center_X, frame_center_Y);
		cv::Point3d ray = cam_model_.projectPixelTo3dRay(frame_center);
		double ray_x = ray.x;
		double ray_y = ray.y;
		double ray_z = ray.z;
		double lidar_x = ray_z;
		double lidar_z = ray.y;
		double lidar_y = ray.x;
		ROS_INFO("Ray (camera coord): xyz (%f,%f,%f)",ray_x,ray_y,ray_z);
		//ROS_INFO("Ray (lidar coord): xyz (%f,%f,%f)",ray_x,ray_y,ray_z);
		double lidar_elevation = lidar_z / lidar_x;
		double lidar_bearing = lidar_y / lidar_x;
		//ROS_INFO("Ray Elevation, Bearing (lidar coord): (%f,%f)",lidar_elevation,lidar_bearing);
		pcl::PointXYZ origin(0,0,0);
		
		int count = 0;
		double average_distance = 0;
		double average_latitude = 0;
    // iterate through point cloud by index
		vector<double> distances(20);
		vector<double> latitudes(20);
		
    for(int i = 0; i < num_points; i++)
    {	
      
			float x = cloud_filtered->points[i].x;
			float y = cloud_filtered->points[i].y;
			float z = cloud_filtered->points[i].z;
			double point_elevation = z / x;
			double point_bearing = y / x;
			//ROS_INFO("Point Elevation, Bearing (lidar co-ord): (%f,%f)",point_elevation,point_bearing);
			
      float distance = calculate_distance(cloud_filtered->points[i], origin);
			
			double elevation_diff = std::abs(lidar_elevation - point_elevation);
			double bearing_diff = std::abs(lidar_bearing - point_bearing);
			
			double tolerance = 0.1;
			if(elevation_diff < tolerance && bearing_diff < tolerance)
			{
				distances.pushback(distance);
				latitudes.pushback(y);
				
				average_distance += distance;
				average_latitude += y;
				count++;
				//ROS_INFO("Point matches azimuth XYZ: (%f,%f,%f); Distance (%f)", x,y,z, distance);
			}
    }
		// generate sorted list of points
		std::sort (distances.begin(), distances.end());
		std::sort (distances.begin(), latitudes.end());
		// create coarse histogram
		int numPoints = distances.size();
		int min = std::floor(distances[0]);
		int max = std::ceil(distances[numPoints-1]);
		int bin_width = 1; //1 meter
		int num_bins = (max - min)/bin_width;
		
		vector<int> bin_indexes(num_bins);
		vector<int> bin_count(num_bins);		
		
		int current_bin = 0;		
		int current_threshold = min + bin_width;
		int max_count = 0;
		int max_bin_index = 0;				
		for(int i = 0; i < numPoints; i++)
		{
			double current_value = distances[i];
			if(floor(current_value)) > current_threshold)
			{
				bin_indexes[current_bin] = i;	
				current_bin +=1;
				current_threshold += bin_width;				
			}
			bin_count[current_bin] += 1;
			// update which histogram bin is the largest
			if(bin_count[current_bin] > max_count)
			{
				max_count = bin_count[current_bin];
				max_bin_index = current_bin;
			}
		}

		// remarks - can check for Nan
		// can construct histogram without sorting
		double winning_distance = max_bin_index * bin_width;
	 
		average_distance /= count;
		average_latitude /= count;
		if(frame_detected)
		{
			//ROS_INFO("Number of matching points: %i", count); 
			//ROS_INFO("Average distance (%f) to [u,v] (%i,%i)",average_distance,frame_center_X,frame_center_Y);
		} else {
			//ROS_INFO("No frame detected"); 
			//average_distance = std::nan("100");
			average_distance = 100; 			
		}
		ROS_INFO("Average distance (%f) average_latitude (%f) to center of image [u,v] (%i,%i)",average_distance, average_latitude,frame_center_X,frame_center_Y);
		/*if(!std::isnan(average_distance) && !std::isnan(old_range))
		{
			range_rate = (old_range - average_distance) * 10 + prev_range_rate / 2;
		} else {
			range_rate = std::nan("100");
		}	*/
		//old_range = average_distance;
		//ROS_INFO("Range(%f) Range Rate (%f)",average_distance,range_rate);
		std_msgs::Float32 dist_msg;
		dist_msg.data = average_distance;
		distancePub.publish(dist_msg);
    // 
  } else {
    ROS_INFO("Camera model not initialized");
  }
}

void frame_cb(const geometry_msgs::Pose2D& pose_msg)
{
	//frame_detected = true;
	frame_center_X = pose_msg.x;
	frame_center_Y = pose_msg.y;
}

void frame_detected_cb(const std_msgs::Bool& frame_detected_msg)
{
	if(frame_detected_msg.data)
	{
		frame_detected = true;
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
		ROS_INFO("Initialized camera model from camera info message");
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
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
		
	// Spin
  ros::spin ();
}
