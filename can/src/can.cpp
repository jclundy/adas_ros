#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// for ctrl-c handling
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

// for can interface
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_signal_defs.h"

static int soc;
static struct can_frame frame_rd;
static int recvbytes = 0;

typedef struct
{
	int can_id;
	int obj_id;
	bool has_range;
	unsigned int range;
} posted_can_msg_t;

posted_can_msg_t vehicles[3] = {
	{
		.can_id = adas_msg_ids[0],
		.obj_id = 0,
		.has_range = false
	},
	{
		.can_id = adas_msg_ids[1],
		.obj_id = 0,
		.has_range = false
	},
	{
		.can_id = adas_msg_ids[2],
		.obj_id = 0,
		.has_range = false
	}
};

double adas_signal_values[adas_signal_count] = {};
double gmhs_signal_values[gmhs_signal_count] = {};

volatile sig_atomic_t stop = 0;

std_msgs::Float32 veh_speed_msg;

ros::Publisher veh_speed_pub;

void inthand(int signum)
{
  stop = 1;
}

int select_can_id(void)
{
	int id = 0;
	unsigned int max_range_idx = 0;
	
	// if object id exists on CAN, select corresponding CAN id
	for(int i = 0; i < sizeof(adas_msg_ids)/sizeof(adas_msg_ids[0]); i++)
	{
		if(adas_signal_values[obj_id] == vehicles[i].obj_id)
		{
			id = vehicles[i].can_id;
			goto fcn_exit;
		}
	}
	
	// select CAN id that has no data, if any (precedence: low id to high id)
	for(int i = 0; i < sizeof(adas_msg_ids)/sizeof(adas_msg_ids[0]); i++)
	{
		if(!vehicles[i].has_range)
		{
			id = vehicles[i].can_id;
			goto fcn_exit;
		}
	}
	
	// find CAN id with furthest vehicle
	for(int i = 1; i < sizeof(adas_msg_ids)/sizeof(adas_msg_ids[0]); i++)
	{
		if(vehicles[i].range > vehicles[max_range_idx].range)
		{
			max_range_idx = i;
		}
	}
	// if current vehicle is closer than the furthest, select CAN id of furthest
	if(adas_signal_values[long_range] < vehicles[max_range_idx].range)
	{
		id = vehicles[max_range_idx].can_id;
	}
	
	fcn_exit:
	
	return id;
}

void assign_values_to_publisher(void)
{
	veh_speed_msg.data = gmhs_signal_values[vehicle_speed];
}

int open_port(const char *port)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  /* open socket */
  soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(soc < 0)
  {
		printf("Fail to open socket");
    return -1;
  }

  addr.can_family = AF_CAN;
  strcpy(ifr.ifr_name, port);

  if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
  {
		printf("Fail ioctl");
    return -1;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  fcntl(soc, F_SETFL, O_NONBLOCK);

  if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
		printf("Fail to bind socket to interface");
    return -1;
  }

  return 0;
}

int send_port(char *payload)
{
  int retval;
  struct can_frame frame = {
  	.can_id = select_can_id(),
  	.can_dlc = 8,
  	.__pad = 0,
  	.__res0 = 0,
  	.__res1 = 0,
  	.data = {
  		payload[0],
  		payload[1],
  		payload[2],
  		payload[3],
  		payload[4],
  		payload[5],
  		payload[6],
  		payload[7]
  	}
  };
 	retval = write(soc, &frame, sizeof(struct can_frame));
  if (retval != sizeof(struct can_frame))
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void compose_can_signal(int signal_enum, double physical_value, char *data)
{
	int raw_value = (physical_value - adas_signal_defs[signal_enum].offset) / adas_signal_defs[signal_enum].scaling;
	int total_bytes = ceil(adas_signal_defs[signal_enum].bit_length / 8.0);
	int start_byte = floor(adas_signal_defs[signal_enum].start_bit / 8.0);
	
	raw_value <<= (adas_signal_defs[signal_enum].start_bit % 8);
	
	int byte = 0;
	int payload[2];
	for(int i = 0; i < (total_bytes-1); i++)
	{
		byte = raw_value & 0b11111111;
		raw_value >>= 8;
		payload[i] = byte;
	}
	
	int bits_in_ms_byte = adas_signal_defs[signal_enum].bit_length - ((8-(adas_signal_defs[signal_enum].start_bit % 8)) + (total_bytes-2)*8);
	int mask = 0b1;
	for(int n = 0; n < (bits_in_ms_byte-1); n++)
	{
 		mask <<= 1;
  	mask |= 1;
	}
	raw_value &= mask;
	byte = raw_value & 0b11111111;
	payload[total_bytes-1] = byte;
	
	int i = start_byte;
	int j = 0;
	while(j < total_bytes)
	{
		data[i] = payload[j];
		if(adas_signal_defs[signal_enum].big_endian)
			i--;
		else
			i++;
		j++;
	}
}

void write_can_adas_signals(void)
{
	char payload[8] = {};
	for(int i = 0; i < adas_signal_count; i++)
	{
		char signal_data[8] = {};
		compose_can_signal(i, adas_signal_values[i], signal_data);
		for(int j = 0; j < 8; j++)
		{
			payload[j] |= signal_data[j];
		}
	}
	
	send_port(payload);
}

void parse_messages(struct can_frame frame)
{
	canid_t id = frame.can_id & CAN_EFF_MASK;
	for(int signal = 0; signal < gmhs_signal_count; signal++)
	{
		if(id == gmhs_signal_defs[signal].can_id)
		{
			int start_byte = floor(gmhs_signal_defs[signal].start_bit / 8.0);
			int total_bytes = ceil(gmhs_signal_defs[signal].bit_length / 8.0);
			char *data = (char*)(&(frame.data[start_byte]));
			char payload[total_bytes];
			int i = 0;
			int j = 0;
			while(i < total_bytes)
			{	
				payload[i] = data[j];
				if(gmhs_signal_defs[signal].big_endian)
					j--;
				else
					j++;
				i++;
			}
			payload[0] >>= (gmhs_signal_defs[signal].start_bit % 8);
			
			int bits_in_ms_byte = gmhs_signal_defs[signal].bit_length - ((8-(gmhs_signal_defs[signal].start_bit % 8)) + (total_bytes-2)*8);
   		if((total_bytes == 1) && (gmhs_signal_defs[signal].start_bit % 8 != 0))
   		{
   			bits_in_ms_byte -= gmhs_signal_defs[signal].start_bit % 8;
   		}
   		
   		int mask = 0b1;
			for(int n = 0; n < (bits_in_ms_byte-1); n++)
			{
	   		mask <<= 1;
	    	mask |= 1;
			}
			payload[total_bytes-1] &= mask;
		
			int physical_value = payload[total_bytes-1];
			for(int p = total_bytes-2; p >= 0; p--)
			{
				physical_value <<= 8;
				physical_value |= payload[p];
			}
			
			int sign = 1;
			if(gmhs_signal_defs[signal].is_signed)
			{
				sign = ((payload[total_bytes-1] >> (bits_in_ms_byte-1)) == 1) ? -1 : 1;
				if(sign == -1)
				{
					physical_value = ~(physical_value - 1) & 0b11111111;
				}
			}
			
			gmhs_signal_values[signal] = sign * ((physical_value * gmhs_signal_defs[signal].scaling) + gmhs_signal_defs[signal].offset);
		}
	}
}

void read_port()
{
	struct timeval timeout = {0, 0};
	fd_set readSet;
	FD_ZERO(&readSet);
	FD_SET(soc, &readSet);

	if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
	{
		if (FD_ISSET(soc, &readSet))
		{
			recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
			if(recvbytes)
			{
				parse_messages(frame_rd);
				assign_values_to_publisher();
			}
		}
	}
}

void obj_id_cb(const std_msgs::Int32& info_msg)
{
	adas_signal_values[obj_id] = info_msg.data;
}

void distance_cb(const std_msgs::Float32MultiArray& info_msg)
{
	adas_signal_values[long_range] = info_msg.data[0];
}

void azimuth_cb(const std_msgs::Float32MultiArray& info_msg)
{
	adas_signal_values[azimuth] = info_msg.data[0];
}

void lateral_distance_cb(const std_msgs::Float32MultiArray& info_msg)
{
	adas_signal_values[lat_range] = info_msg.data[0];
}

void relative_lane_cb(const std_msgs::Int32MultiArray& info_msg)
{
	adas_signal_values[rel_lane] = info_msg.data[0];
}

void range_rate_cb(const std_msgs::Float32MultiArray& info_msg)
{
	adas_signal_values[range_rate] = info_msg.data[0];
}

void brake_light_cb(const std_msgs::Int32& info_msg)
{
	adas_signal_values[brake_light] = info_msg.data;
}

void obj_age_cb(const std_msgs::Int32& info_msg)
{
	adas_signal_values[obj_age] = info_msg.data;
}

int main(int argc, char** argv)
{
	//Ros node and publisher initialization
	ros::init(argc, argv, "can");
	ros::NodeHandle nh;

	signal(SIGINT, inthand);

	// select whatever interface is wired up on the Jetson
	open_port("can1");

	ros::Subscriber obj_id_sub = nh.subscribe("/darknet_ros/obj_id",1,obj_id_cb);
	ros::Subscriber distance_sub = nh.subscribe("/lidar_ranging/distance",1, distance_cb);	 
	ros::Subscriber azimuth_sub = nh.subscribe("/lidar_ranging/azimuth",1, azimuth_cb);
	ros::Subscriber lateral_distance_sub = nh.subscribe("/lidar_ranging/lateral_distance",1, lateral_distance_cb);
	ros::Subscriber relative_lane_sub = nh.subscribe("/lidar_ranging/relative_lane",1, relative_lane_cb);
	ros::Subscriber range_rate_sub = nh.subscribe("/lidar_ranging/range_rate",1,range_rate_cb);
	ros::Subscriber brake_light_sub = nh.subscribe("/darknet_ros/brake_light",1,brake_light_cb);
	ros::Subscriber obj_age_sub = nh.subscribe("/darknet_ros/obj_age",1,obj_age_cb);
	
	veh_speed_pub = nh.advertise<std_msgs::Float32>("/can/vehicle_speed",1); 

	while(!stop)
	{
		read_port();
		veh_speed_pub.publish(veh_speed_msg);
		
		ros::spinOnce();
		
		write_can_adas_signals();
	}
}

