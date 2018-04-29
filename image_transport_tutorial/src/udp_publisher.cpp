#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <chrono>
#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <iostream>          // For cout and cerr
#include <cstdlib>           // For atoi()
#include "s32v_video.h"

#define MAX_OBJECTS_PER_FRAME (100)
#define BUF_LEN 65540

int main(int argc, char** argv)
{
	//Ros node and publisher initialization
	ros::init(argc, argv, "udp_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/videofile/image_raw", 1);

	//Start of UDP streaming
	unsigned short servPort = 7777;

  try {
    UDPSocket sock(servPort);

		char buffer[BUF_LEN]; // Buffer for echo string
    int recvMsgSize; // Size of received message
    string sourceAddress; // Address of datagram source
    unsigned short sourcePort; // Port of datagram source
		
		ros::Rate loop_rate(30);

		while (nh.ok()) {
			
			do {
			    recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
		  } while (recvMsgSize > sizeof(int));
	      
      int total_pack = ((int * ) buffer)[0];

      if (UDP_DEBUG){
        cout << "expecting length of packs:" << total_pack << endl;
      }

      char * longbuf = new char[PACK_SIZE * total_pack];

      for (int i = 0; i < total_pack; i++) {
        recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
        if (recvMsgSize != PACK_SIZE) {
            cerr << "Received unexpected size pack:" << recvMsgSize << endl;
            continue;
        }
        memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
      }

      //auto frameCaptureTime = std::chrono::system_clock::now();

      if (UDP_DEBUG){
          cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;
      }

      cv::Mat rawData = Mat(1, PACK_SIZE * total_pack, CV_8UC1, longbuf);
      cv::Mat image = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
      if (image.size().width == 0) {
        cerr << "decode failure!" << endl;
        continue;
      }

      free(longbuf);

      if( image.empty() ) {
        ROS_INFO("image.empty error\n");
        waitKey();
        return -1;
      }
      else {
				//publish image
				sensor_msgs::ImageConstPtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			  pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}	
	
		}
	} catch (SocketException & e) {
		ROS_INFO("Socket exception");
		cerr << e.what() << endl;
		return -1;
	} 
}

