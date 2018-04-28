#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cmath>
#include <stdlib.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 400
#define LIDAR_MAX_RANGE 100

#define CENTER_X 320
#define CENTER_Y 200

std::string frame_id = "velodyne";
std::string ns = "points_and_lines";

double ray_x = 1;
double ray_y = 0;
double ray_z = 0;

image_geometry::PinholeCameraModel cam_model_;
sensor_msgs::CameraInfo camera_info_msg;

int frame_center_X = 320;
int frame_center_Y = 150;

bool frame_detected = false;
bool frame_has_appeared = false;
int no_detection_count = 0;
bool camera_model_initialized = false;

cv::Point3d camera_position;
double theta_y = 0.1;
double camera_position_x = -1.872;
double camera_position_y = 0;
double camera_position_z = 0.655; 

void frame_cb(const geometry_msgs::Pose2D& pose_msg)
{
  //frame_detected = true;
  if(frame_detected)
  {
    frame_center_X = pose_msg.x;
    frame_center_Y = pose_msg.y;
    /*
		cv::Point2d frame_center = cv::Point2d(frame_center_X, frame_center_Y);
    cv::Point3d ray_cam = cam_model_.projectPixelTo3dRay(frame_center);

    // rotate the ray
    double ray_world_x = ray_cam.z;
    double ray_world_z = ray_cam.y;
    double ray_world_y = ray_cam.x;
    ray_x = std::cos(theta_y)*ray_world_x + std::sin(theta_y)*ray_world_z;
    ray_z = -std::sin(theta_y)*ray_world_x + std::cos(theta_y)*ray_world_z;
    ray_y = ray_world_y;
		*/
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
  if(!camera_model_initialized)
  {
    cam_model_.fromCameraInfo(*info_msg);
		camera_model_initialized = true;
		ROS_INFO("camera model intialized");
  }
}

void draw_line(int frame_center_X, int frame_center_Y, image_geometry::PinholeCameraModel cam_model_, ros::Publisher marker_pub)
{
		
		cv::Point2d frame_center = cv::Point2d(frame_center_X, frame_center_Y);
	  cv::Point3d ray_cam = cam_model_.projectPixelTo3dRay(frame_center);

	  // rotate the ray
		ROS_INFO("cv ray (xyz) : (%f, %f, %f)",ray_cam.x, ray_cam.y, ray_cam.z);
	  double ray_world_x = ray_cam.z;
	  double ray_world_z = ray_cam.y;
	  double ray_world_y = ray_cam.x;
		ROS_INFO("world ray (xyz) : (%f, %f, %f)",ray_world_x, ray_world_z, ray_world_y);
	
	  ray_x = std::cos(theta_y)*ray_world_x + std::sin(theta_y)*ray_world_z;
	  ray_z = -std::sin(theta_y)*ray_world_x + std::cos(theta_y)*ray_world_z;
	  ray_y = ray_world_y;
	
		ROS_INFO("rotated ray (xyz) : (%f, %f, %f)",ray_x, ray_z, ray_y);
		//

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

	  // Create the vertices for the points and lines
	  double myx = ray_y / ray_x;
	  double mzx = ray_z / ray_x;

	  for (uint32_t i = 0; i < 10; ++i)
	  {
	    //draw every 1 meter
	    float dx = i;
	    float dy = dx*myx;
	    float dz = dx*mzx;

	    geometry_msgs::Point p;
	    p.x = camera_position_x + dx;
	    p.y = camera_position_y + dy;
	    p.z = camera_position_z + dz;

	    points.points.push_back(p);
	    line_strip.points.push_back(p);
	  } 
  	marker_pub.publish(points);
  	marker_pub.publish(line_strip);
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Subscriber camera_info_sub = nh.subscribe("/videofile_test/camera_info", 1, camera_cb);
  
  // Create ROS subscriber for frame_center_sub
  ros::Subscriber frame_center_sub = nh.subscribe("/darknet_ros/frame_center",1,frame_cb);
  ros::Subscriber frame_detected_sub = nh.subscribe("/darknet_ros/frame_detected",1,frame_detected_cb);

 	ros::Rate r(30);
	
	if(argc > 1) {
		theta_y = std::atof(argv[1]);
	}
  
	while (ros::ok())
  {
		if(camera_model_initialized)
		{
			draw_line(frame_center_X, frame_center_Y, cam_model_, marker_pub);	
		}	
	  ros::spinOnce ();
    r.sleep();
  }
}

/*
to think about
- should capture a certain range of data to loop, from both point cloud data and 
from a time frame where the bounding box is present 
- add subscriber and callback fn for bounding box center
- add subscriber for camera data
- run with different videos, to verify 3d vector direction corresponds to orientation in 2d video feed
*/
