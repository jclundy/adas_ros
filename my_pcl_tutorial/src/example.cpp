#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
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


#define CAMERA_PIXEL_WIDTH 1222
#define CAMERA_PIXEL_HEIGHT 334
#define LIDAR_MAX_RANGE 100
#define GRAY_MAX 255
#define RANGE_TO_GRAY_RATIO 2.55// = 255/100

#define U_OFFSET 499
#define V_OFFSET 92

int frame_center_X = 320;
int frame_center_Y = 200;

//U : (max,min) (723,-499); V: (max,min) (242,-92)
// U : (max,min) (723,-499); V: (max,min) (242,-92)
// : (max,min) (723,-499); V: (max,min) (244,-92)
//publishers
ros::Publisher pub;
ros::Publisher pub_2;

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

void initialize_range_pixels()
{
	range_pixels = LIDAR_MAX_RANGE*cv::Mat::eye(CAMERA_PIXEL_HEIGHT, CAMERA_PIXEL_WIDTH,CV_32F);
	range_gray_pixels = GRAY_MAX*cv::Mat::eye(CAMERA_PIXEL_HEIGHT, CAMERA_PIXEL_WIDTH,CV_8U);
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
	
  initialize_range_pixels();

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
		//ROS_INFO("Ray (camera coord): xyz (%f,%f,%f)",ray_x,ray_y,ray_z);
		//ROS_INFO("Ray (lidar coord): xyz (%f,%f,%f)",ray_x,ray_y,ray_z);
		double lidar_elevation = lidar_z / lidar_x;
		double lidar_bearing = lidar_y / lidar_x;
		//ROS_INFO("Ray Elevation, Bearing (lidar coord): (%f,%f)",lidar_elevation,lidar_bearing);
		pcl::PointXYZ origin(0,0,0);
		
		int count = 0;
		double average_distance = 0;

    // iterate through point cloud by index
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
			

			if(elevation_diff < 0.1 && bearing_diff < 0.1)
			{
				average_distance += distance;
				count++;
				//ROS_INFO("Point matches azimuth XYZ: (%f,%f,%f); Distance (%f)", x,y,z, distance);
			}
    }
		average_distance /= count;
		ROS_INFO("Number of matching points: %i", count); 
		ROS_INFO("Average distance (%f) to [u,v] (%i,%i)",average_distance,frame_center_X,frame_center_Y);
    // 
  } else {
    ROS_INFO("Camera model not initialized");
  }
	
	// Publish range image
	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, range_gray_pixels).toImageMsg();
	range_image_pub.publish(img_msg);
}

void frame_cb(const geometry_msgs::Pose2D& pose_msg)
{
	frame_center_X = pose_msg.x;
	frame_center_Y = pose_msg.y;
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
	
  //Initialize range image matrixes
	initialize_range_pixels();
  
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
  
	// Spin
  ros::spin ();
}
