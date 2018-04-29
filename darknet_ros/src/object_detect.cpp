#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "darknet_ros/DetectedObjects.h"
#include "darknet_ros/ObjectInfo.h"
#include "darknet_ros/ImageDetection.h"


#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>

#include "arapaho.hpp"
#include <string>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <utility>

// c++ std
#include <math.h>
#include <chrono>
#include <vector>
#include <ctime>

#define OUTPUT_VIDEO							0
#define SHOW_VIDEO								1
#define PRINT_FPS									0

#define YOLO_SAMPLING_FRAME_COUNT 1
#define MAX_OBJECTS_PER_FRAME 		100
#define MAX_INVALID_DETECTED_AREA 85000
#define SIZE_SHAPE_THRESHOLD			0.50

using namespace cv;

ArapahoV2* p;
float thresh;
ros::Publisher obj_id_pub;
ros::Publisher brake_light_pub;
ros::Publisher obj_age_pub;
ros::Publisher objPub_;
ros::Publisher frameCenterPub;
ros::Publisher frameDetectedPub;
image_transport::Publisher imgPub_;
ros::ServiceServer yoloSrv_;
std::chrono::system_clock::time_point frameCaptureTime;

Rect2d newRect[MAX_OBJECTS_PER_FRAME] = {Rect(0,0,0,0)};
std::vector<Ptr<Tracker>> trackers;
std::vector<Rect2d> bbox;
std::set<int> tracked_ids;
bool tracker_init = false;

std::vector<float> obj_age;

double range = 0;
double old_range = 0;
double range_rate = 0;
double prev_range_rate = 0;
int relative_lane = 0;

int frameCount = 0;
std::chrono::duration<double> fps(17);

int centerX = 0;
int centerY = 0;

int current_time;
int prev_time;

int not_detected_count = 0;
bool frame_is_detected = false;
bool new_detection_frame = false;

int frame_width = 640;
int frame_height = 400;
VideoWriter outputVideo;

//bgr
const Scalar green(0,128,0);
const Scalar yellow(0,255,255);
const Scalar red(0,0,255);

std::pair<darknet_ros::DetectedObjects, cv_bridge::CvImagePtr> detectImage(sensor_msgs::Image msg)
{
  try
  {
    int numObjects = 0;
    bool ret = false;
    int expectedW = 0, expectedH = 0;
    box* boxes = 0;
		
    std::string* labels;
    //cv_ptr_  = cv_bridge::toCvShare(msg, "bgr8");
    cv_bridge::CvImagePtr cv_ptr_ = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
		
		// Remember the time
    frameCaptureTime = std::chrono::system_clock::now();
		
    if( cv_ptr_.get()) 
    {
	    darknet_ros::DetectedObjects tObjMsg;
	    tObjMsg.header = cv_ptr_->header;
	    
      if(frameCount % YOLO_SAMPLING_FRAME_COUNT == 0){
		    int imageWidthPixels = cv_ptr_->image.size().width;
		    int imageHeightPixels = cv_ptr_->image.size().height;
		    DPRINTF("Image data = %p, w = %d, h = %d\n", cv_ptr_->image.data, imageWidthPixels, imageHeightPixels);
		    
		    p->Detect(
		      cv_ptr_->image,
		      0.3,
		      0.5,
		      numObjects);

				new_detection_frame = false;

		    if(numObjects > 0)
		    { 		      
		      boxes = new box[numObjects];
		      labels = new std::string[numObjects];
		   
		      // Get boxes and labels
		      p->GetBoxes(
		        boxes,
		        labels,
		        numObjects
		        );

		      int objId = 0;
		      int leftTopX = 0, leftTopY = 0, rightBotX = 0,rightBotY = 0;
		      tracked_ids.clear();
		      for (objId = 0; objId < numObjects; objId++)
		      {
		        if(labels[objId] == "bus" || labels[objId] == "car" || labels[objId] == "motorbike")
		        {
							not_detected_count = 0;        
							frame_is_detected = true;
							new_detection_frame = true;
				      
							centerX = imageWidthPixels*boxes[objId].x;
							centerY = imageHeightPixels*boxes[objId].y;
							//printf("image center: (%i, %i)\n",centerX, centerY);
				        // Show labels
				      if (labels[objId].c_str())
				      {		        	
			          leftTopX = 1 + imageWidthPixels*(boxes[objId].x - boxes[objId].w / 2);
			          leftTopY = 1 + imageHeightPixels*(boxes[objId].y - boxes[objId].h / 2);
			          rightBotX = 1 + imageWidthPixels*(boxes[objId].x + boxes[objId].w / 2);
			          rightBotY = 1 + imageHeightPixels*(boxes[objId].y + boxes[objId].h / 2);
				        DPRINTF("Label:%s\n\n", labels[objId].c_str());
								newRect[objId].x = leftTopX;
								newRect[objId].y = leftTopY;
								newRect[objId].width = rightBotX - leftTopX;
				    	  newRect[objId].height = rightBotY - leftTopY;
				    	  if(trackers.size() > 0){
									for(int i = 0; i < trackers.size(); i++)
									{										
									
										if(frameCount % (int)fps.count() == 0){
											old_range = range;
											if(!std::isnan(range) && !std::isnan(old_range))
											{
												range_rate = range - old_range;
											} 
											else 
											{
												range_rate = std::nan("1");
											}										
										}
									
										if((newRect[objId].area() < MAX_INVALID_DETECTED_AREA)){
											int overlapArea = (newRect[objId] & bbox[i]).area();									    		
								  		int old_centroid_to_vertex_x = bbox[i].width/2;
								  		int old_centroid_to_vertex_y = bbox[i].height/2;
								  		double old_centroid_to_vertex = sqrt((old_centroid_to_vertex_x*old_centroid_to_vertex_x)+(old_centroid_to_vertex_y*old_centroid_to_vertex_y));
								  		
								  		int old_centroid_x = bbox[i].x + bbox[i].width/2;
								  		int old_centroid_y = bbox[i].y + bbox[i].height/2;
								  		int new_centroid_x = newRect[objId].x + newRect[objId].width/2;
								  		int new_centroid_y = newRect[objId].y + newRect[objId].height/2;
								  		
								  		int centroid_to_centroid_x = old_centroid_x - new_centroid_x;
								  		int centroid_to_centroid_y = old_centroid_y - new_centroid_y;
								  		double centroid_to_centroid = sqrt((centroid_to_centroid_x*centroid_to_centroid_x)+(centroid_to_centroid_y*centroid_to_centroid_y));
										
											if((overlapArea > bbox[i].area()/4) || centroid_to_centroid < old_centroid_to_vertex)
											{
												bbox[i] = Rect2d(newRect[objId].x, newRect[objId].y, newRect[objId].width, newRect[objId].height);
												trackers[i]->clear();
												trackers[i]->init(cv_ptr_->image, bbox[i]);
												tracked_ids.insert(i);
												goto set_track_init;
							  			}
										}
									
									}
								}
							
								bbox.push_back(Rect2d(newRect[objId].x, newRect[objId].y, newRect[objId].width, newRect[objId].height));
								trackers.push_back(TrackerKCF::create());
								trackers.back()->init(cv_ptr_->image, bbox.back());
								tracked_ids.insert(trackers.size()-1);
							
								set_track_init:
							
								tracker_init = true;
		        	}
						}
		      } 
					if(!new_detection_frame) 
					{ 
						not_detected_count++;
					}
		      if (boxes)
		      {
		        delete[] boxes;
		        boxes = NULL;
		      }
		      if (labels)
		      {
		        delete[] labels;
		        labels = NULL;
		      }   

		    }// If objects were detected
      }
    
      
      darknet_ros::ObjectInfo newObj;
      std::vector<darknet_ros::ObjectInfo> objectList;
      
      for(int i = 0; i < trackers.size(); i++)
      {
				if(tracker_init)
				{
					if((tracked_ids.find(i) == tracked_ids.end())){
						continue;
					}						
					trackers[i]->update(cv_ptr_->image, bbox[i]);
					if(bbox[i].area() < MAX_INVALID_DETECTED_AREA){
			      newObj.tl_x = bbox[i].x < 0 ? 0 : bbox[i].x;
			      newObj.tl_y = bbox[i].y < 0 ? 0 : bbox[i].y;
			      newObj.width = bbox[i].width;
			      newObj.height = bbox[i].height;
			      newObj.type = "car";
			      objectList.push_back(newObj);
			      
						putText(cv_ptr_->image, "car: "+std::to_string(i+1), cvPoint(bbox[i].x, bbox[i].y-40),
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						putText(cv_ptr_->image, "range: "+std::to_string(range), cvPoint(bbox[i].x, bbox[i].y-20),
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						putText(cv_ptr_->image, "range rate: "+std::to_string(range_rate), cvPoint(bbox[i].x, bbox[i].y),
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						
						Scalar box_colour = green;
						if(relative_lane == 0)
						{
							if(range > 25)
							{
								box_colour = yellow;
							}
							else
							{
								box_colour = red;
							}
						}
						rectangle(cv_ptr_->image, bbox[i], box_colour,2,8,0);
						
						std_msgs::Int32 obj_id_msg;
						obj_id_msg.data = i + 1;
						obj_id_pub.publish(obj_id_msg);
						
						if(obj_age.size() > i)
						{
							obj_age.at(i) += 1;
						}
						else
						{
							obj_age.push_back(1);
						}
						
						int j = 0;
						while((j < obj_age.size()) && (j != i))
						{
							obj_age.at(j) = 0;
							j++;
						}
						
						std_msgs::Int32 obj_age_msg;
						obj_age_msg.data = obj_age.at(i) / fps.count();
						obj_age_pub.publish(obj_age_msg);
						
						int brake_light = 0;
						Mat img;
						// convert to lab colour space
    				cvtColor(cv_ptr_->image(bbox[i]), img, CV_BGR2Lab);
    				// threshold the image
						inRange(img, Scalar(0,150,100), Scalar(255,200,153),img);
						
						morphologyEx(img,img,MORPH_GRADIENT,getStructuringElement(MORPH_ELLIPSE, Size(7,7)));
						morphologyEx(img,img,MORPH_OPEN,getStructuringElement(MORPH_ELLIPSE, Size(4,4)));
						morphologyEx(img,img,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
						
						Mat labels, stats, centroids;
						int num = connectedComponentsWithStats(img,labels,stats,centroids,8,CV_32S);
						
						std::vector< std::vector<Rect2d> > candidate_pairs;
						if(num < 2)
						{
							// unknown brake light
							brake_light = 0;
						}
						else
						{
							// y-distance test
							for(int m = 1; m < num; m++)
							{
								for(int n = m+1; n <= num; n++)
								{
									int y_dist_centroid = abs(centroids.at<double>(m,1) - centroids.at<double>(n,1));
									if((y_dist_centroid <= stats.at<double>(m,CC_STAT_HEIGHT)) || (y_dist_centroid <= stats.at<double>(n,CC_STAT_HEIGHT)))
									{
										std::vector<Rect2d> pair = {
											Rect2d(
												stats.at<int>(m,CC_STAT_LEFT),
												stats.at<int>(m,CC_STAT_TOP),
												stats.at<int>(m,CC_STAT_WIDTH),
												stats.at<int>(m,CC_STAT_HEIGHT)
											), 
											Rect2d(
												stats.at<int>(n,CC_STAT_LEFT),
												stats.at<int>(n,CC_STAT_TOP),
												stats.at<int>(n,CC_STAT_WIDTH),
												stats.at<int>(n,CC_STAT_HEIGHT)
											)
										};
										candidate_pairs.push_back(pair);
									}
								}
							}
							
							// size-shape test
							int num_erased = 0;
							for(int k = 0; k < candidate_pairs.size(); k++)
							{
								Rect2d first = candidate_pairs[k][0];
								Rect2d second = candidate_pairs[k][1];
								int first_centroid_x = first.x + first.width/2.0;
								int first_centroid_y = first.y + first.height/2.0;

								second.x = first_centroid_x - second.width/2.0;								
								second.y = first_centroid_y - second.height/2.0;
								
								double intersection_area = (first & second).area();
								double union_area = (first | second).area();
								double G = intersection_area / union_area;
								
								if(G < SIZE_SHAPE_THRESHOLD)
								{
									candidate_pairs.erase(candidate_pairs.begin() + k - num_erased);
									num_erased++;
								}
								else
								{
									candidate_pairs[k].push_back(Rect2d(G,0,0,0));
								}
							}
							
							// pair selection
							double pairing_score = 0;
							double total_pair_area = 0;
							for(int k = 0; k < candidate_pairs.size(); k++)
							{
								total_pair_area += candidate_pairs[k][0].area();
								total_pair_area += candidate_pairs[k][1].area();
							}
							int best_pair = 0;
							int max_pairing_score = 0;
							for(int k = 0; k < candidate_pairs.size(); k++)
							{
								double pair_area = candidate_pairs[k][0].area() + candidate_pairs[k][1].area();
								double H = pair_area / total_pair_area;
								
								pairing_score = (0.5*(candidate_pairs[k][2].x)) + ((1-0.5)*H);
								if(pairing_score > max_pairing_score)
								{
									best_pair = k;
								}
							}
							
							
							
							
						}
						
    				//imshow("CAR", cv_ptr_->image(bbox[i]));
    				
    				std_msgs::Int32 brake_light_msg;
						brake_light_msg.data = brake_light;
						brake_light_pub.publish(brake_light_msg);
					}
				}
      }
			
	    tObjMsg.objects = objectList;
      
      std::chrono::duration<double> fps = (std::chrono::system_clock::now() - frameCaptureTime);

			frameCount++;

      //cv_ptr_.reset();
      return std::make_pair(tObjMsg, cv_ptr_);
//       objPub_.publish(tObjMsg);
//       imgPub_.publish(cv_ptr_->toImageMsg());
//       waitKey(30);
//       cv_ptr_.reset();
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg.encoding.c_str());
    return std::make_pair(darknet_ros::DetectedObjects(), cv_bridge::CvImagePtr());
  }
}

bool imageDetectionService(darknet_ros::ImageDetectionRequest &req, darknet_ros::ImageDetectionResponse &resp)
{
    std::pair<darknet_ros::DetectedObjects, cv_bridge::CvImagePtr> result = detectImage(req.msg);
    resp.objects = result.first;
    resp.img = *(result.second->toImageMsg());
    
    if(resp.objects.objects.size())
  return true;
    else
  return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
    std::pair<darknet_ros::DetectedObjects, cv_bridge::CvImagePtr> result = detectImage(*msg);
		geometry_msgs::Pose2D frame_center;
		frame_center.x = centerX;
		frame_center.y = centerY;   
		frameCenterPub.publish(frame_center);
				
		
		/*if(not_detected_count > 10) {
			not_detected_count = 0;
			frame_is_detected = false;
		}*/
		std_msgs::Bool frameDetectedMsg;
		frameDetectedMsg.data =  new_detection_frame;		
		frameDetectedPub.publish(frameDetectedMsg);	
		
    objPub_.publish(result.first);
    imgPub_.publish(result.second->toImageMsg());
    
    if(OUTPUT_VIDEO)
    	outputVideo.write(result.second->image);
    if(SHOW_VIDEO)
	    imshow("UWAFT", result.second->image);
    
    fps = (std::chrono::system_clock::now() - frameCaptureTime);	    
    if(PRINT_FPS)
    	printf("==> FPS: %f\n", 1/(fps.count()));
    waitKey(1);
}

void distance_cb(const std_msgs::Float32& msg_data)
{
	old_range = range;
	range = msg_data.data;
	
	if(!std::isnan(range) && !std::isnan(old_range))
	{
		range_rate = (old_range - range) * 10 + prev_range_rate / 2;
	} else {
		range_rate = std::nan("1");
	}	
}

void relative_lane_cb(const std_msgs::Int32& msg_data)
{
	relative_lane = msg_data.data;
}

int main(int argc, char **argv)
{
	system("sudo nvpmodel -m 0"); //MAX-N mode
  system("sudo ~/jetson_clocks.sh"); //Max Clock Speeds
  
  // get current time for output file name
  std::time_t t = std::time(0);
  std::tm* now = std::localtime(&t);
  std::string unique_file_name = std::to_string(now->tm_year+1900) +"-"+ 
																 std::to_string(now->tm_mon+1) +"-"+ 
																 std::to_string(now->tm_mday) +"-"+ 
																 std::to_string(now->tm_hour) +"-"+ 
																 std::to_string(now->tm_min) +"-"+ 
																 std::to_string(now->tm_sec);
  outputVideo.open("/media/nvidia/37d85b64-6ae4-4913-beb8-75f3371b5284/UWprocessed_"+unique_file_name+".mp4",CV_FOURCC('M','P','E','G'),20, Size(frame_width,frame_height));

	ros::init(argc, argv, "image_listener");

  ros::NodeHandle nh;
  namedWindow("view");
  startWindowThread();
  
  objPub_ = nh.advertise<darknet_ros::DetectedObjects>( "/darknet_ros/detected_objects", 1);
	frameCenterPub = nh.advertise<geometry_msgs::Pose2D>("/darknet_ros/frame_center",1);
	frameDetectedPub = nh.advertise<std_msgs::Bool>("/darknet_ros/frame_detected",1);
  std::string ros_path = ros::package::getPath("darknet_ros");

	ros::Subscriber distance_sub = nh.subscribe("/lidar_ranging/distance",1,distance_cb);  
	ros::Subscriber relative_lane_sub = nh.subscribe("/lidar_ranging/relative_lane",1,relative_lane_cb);
	
  ros::NodeHandle priNh( "~" );
  std::string yoloWeightsFile;
  std::string yoloConfigFile;
  std::string yoloDataFile;
  
  priNh.param<std::string>( "weights_file", yoloWeightsFile, ros_path + "/tiny-yolo-voc.weights");
  priNh.param<std::string>( "cfg_file", yoloConfigFile, ros_path + "/darknet/cfg/tiny-yolo-voc.cfg");
  priNh.param<std::string>( "name_file", yoloDataFile, ros_path + "/darknet/data/voc.names" );
  priNh.param( "thresh", thresh, 0.2f );
  
  // Initialize darknet object using Arapaho API
  p = new ArapahoV2();
  if(!p)
  {
    return -1;
  }

    // TODO - read from arapaho.cfg    
  ArapahoV2Params ap;

  ap.datacfg = (char *)yoloDataFile.c_str();//INPUT_DATA_FILE.c_str();
  ap.cfgfile = (char *)yoloConfigFile.c_str();//INPUT_CFG_FILE.c_str();
  ap.weightfile = (char *)yoloWeightsFile.c_str();//INPUT_WEIGHTS_FILE.c_str();

  ap.nms = 0.4;
  ap.maxClasses = 20;
  int expectedW = 0, expectedH = 0;
    // Always setup before detect
  bool ret = p->Setup(ap, expectedW, expectedH);
  if(false == ret)
  {
    EPRINTF("Setup failed!\n");
    if(p) delete p;
    p = 0;
    return -1;
  }

  image_transport::ImageTransport it(nh);  

  imgPub_ = it.advertise( "/darknet_ros/image", 1);
  image_transport::Subscriber sub = it.subscribe("/videofile/image_raw", 1, imageCallback);
  
  yoloSrv_ = nh.advertiseService("/darknet_ros/detect_objects", imageDetectionService);

	obj_id_pub = nh.advertise<std_msgs::Int32>("/darknet_ros/obj_id", 1);
	brake_light_pub = nh.advertise<std_msgs::Int32>("/darknet_ros/brake_light", 1);
	obj_age_pub = nh.advertise<std_msgs::Int32>("/darknet_ros/obj_age", 1);
  
  ros::spin();
  if(p) delete p;
  DPRINTF("Exiting...\n");
  return 0;
}
