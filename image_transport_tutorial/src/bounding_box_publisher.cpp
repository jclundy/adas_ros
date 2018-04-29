#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

geometry_msgs::Point32 generate_point_message(cv::Point cv_point)
{
	geometry_msgs::Point32 p_msg;
	p_msg.x = cv_point.x;
	p_msg.y = cv_point.y; 
	return p_msg;
}

geometry_msgs::Polygon generate_polygon_message(cv::Point p1_cv, cv::Point p2_cv)
{
	geometry_msgs::Point32 p1 = generate_point_message(p1_cv);
	geometry_msgs::Point32 p2 = generate_point_message(p2_cv);
	geometry_msgs::Polygon rect;
	rect.points.push_back(p1);
	rect.points.push_back(p2);
	return rect;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
	ros::Publisher box_pub = nh.advertise<geometry_msgs::Polygon> ("camera/bounding_box", 1);
	
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	int width = image.cols;
	int height = image.rows;
	int cx = width/2;
	int cy = height/2;
	int rect_width = int(0.4*width);
	int rect_height = int(0.4*height);
	cv::Point p1(cx - rect_width/2, cy - rect_height/2);
	cv::Point p2(cx + rect_width/2, cy + rect_height/2);

	cv::rectangle(image, p1, p2, cv::Scalar(255,0,0));
	
	geometry_msgs::Polygon rect = generate_polygon_message(p1, p2);	
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
		box_pub.publish(rect);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

