#!/bin/bash
xterm -hold -e "roslaunch darknet_ros object_detect.launch" & 
xterm -hold -e "rosbag play /media/nvidia/37d85b64-6ae4-4913-beb8-75f3371b5284/VehAppr1.bag -l" &
xterm -hold -e "rosbag play /media/nvidia/37d85b64-6ae4-4913-beb8-75f3371b5284/camera_info.bag -l" &
xterm -hold -e "rosrun my_pcl_tutorial example"
