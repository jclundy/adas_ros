#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 400
#define LIDAR_MAX_RANGE 100

#define CENTER_X 320
#define CENTER_U 200

string frame_id = "/ray_vizualize";
string ns = "points_and_lines";
geometry_msgs::Point ray;

ray.x = 1;
ray.y = 0;
ray.z = 0;

geometry_msgs::Point camera_position;
camera_position.x = -1.872;
camera_position.y = 0;
camera_position.z = 0.655; 

int cx = CENTER_X;
int cy = CENTER_Y;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  while (ros::ok())
  {

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
    myx = ray.y / ray.x;
    mzx = ray.z / ray.x;

    for (uint32_t i = 0; i < 10; ++i)
    {
      //draw every 1 meter
      float dx = i;
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