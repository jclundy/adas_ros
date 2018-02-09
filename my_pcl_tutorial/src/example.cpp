#include <ros/ros.h>
// CV specific includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>


ros::Publisher pub;
image_transport::Publisher range_image_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Convert message to pcl::PointCloud type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Apply passthrough filter
	pcl::PassThrough<pcl::PointXYZ> pass;

	pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.0, 20);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-10, 10);
	pass.filter(*cloud_filtered);
	
	// Apply voxel grid	
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
  //sor.setInputCloud (cloud_filtered);
  //sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //sor.filter (*cloud_filtered);

	// Create Range Image
	
	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage range_image;
  range_image.createFromPointCloud(*cloud_filtered, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	// Range image visualization	
	float* ranges = range_image.getRangesArray ();
	float min_value = 0;
	float max_value = 1;
	bool grayscale = 0;
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, range_image.width, range_image.height, min_value, max_value, grayscale);
	cv::Mat frame =  cv::Mat(range_image.width,range_image.height, CV_8UC3, rgb_image);
	
	// Publish point cloud
  sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
  pub.publish (output);

	// Publish range image
	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	range_image_pub.publish(img_msg);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
	
	image_transport::ImageTransport it(nh);
	range_image_pub = it.advertise("range_image",1);

  // Spin
  ros::spin ();
}
