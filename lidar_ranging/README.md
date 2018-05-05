## Setting up ROS on the jetson
### Initial Install
Follow the instructions here: https://wiki.ros.org/kinetic/Installation/Ubuntu

When you get to step 1.4, do the desktop-full install:

`sudo apt-get install ros-kinetic-desktop-full`
### Additional libraries to install
In general: `sudo apt-get install ros-kinetic-LIBRARY_NAME`
#### vlp-16
`sudo apt-get install ros-kinetic-velodyne`
#### pcl
This is likely already installed with the desktop version of ros
https://wiki.ros.org/pcl
#### video_stream_opencv
https://wiki.ros.org/video_stream_opencv
(Optional) This can be used to visualize the camera stream from a video file.  It can also be used to publish camera calibration info from a file to a ros topic
If you do want to use this to view video, you will need to edit the file video_file.launch
To do this:
- `roscd video_stream_opencv`
- `cd launch`
- sudo gedit `video_file.launch`
- change the video_stream_provider to the absolute path of the video file
- change the camera_info_url to the lidar_ranging/ost.yaml, which contains camera metainfo
To run:
`rosrun video_stream_opencv video_file.launch`

#### camera calibration (for distortion calibration)
it may be missing some dependencies, so run
`rosdep install camera_calibration`

## Compiling and building the project
- Create the workspace
Follow the instructions here https://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Add `source ~/catkin_ws/devel/setup.bash` to the .bashrc file
- Building all packges (under catkin_ws/src): `cd ~/catkin_ws` then  `catkin_make`
- Building specific package: `catkin_make --pkg PACKAGE_NAME`
- Clean command `catkin_make clean`

## Setting up the VLP-16
1. Follow the setup instructions here
https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
2. To view the live lidar data:
- `roslaunch velodyne_pointcloud VLP16_points.launch`
- `rosrun rviz rviz -f velodyne`
- in the rviz window, click "Add" -> "By topic" -> /velodyne_points
3. Trouble shooting
- Check the power / ethernet connections
- Try connecting through an ethernet switch (on linux)
- Check the sensors diagnostics by going to http://192.168.1.201 in the browser
- Make sure the ip configuration is setup correctly
- See if it works in Windows:
-- Manual IP config http://velodynelidar.com/docs/manuals/63-9266%20REV%20A%20WEBSERVER%20USER%20GUIDE,HDL-32E%20&%20VLP-16.pdf
-- Install veloview
-- run veloview, and click "open" -> "sensor stream"

## Running Everything
1. connect to s32v234 and run video streaming
- `ssh root@192.168.1.10`
- `./videoStream.elf`
2. start the camera streaming script on the jetson
- `rosrun image_transport_tutorial udp_publisher`
3. Run object detection code
- `roslaunch darknet_ros object_detect`
4. Run the camera data publisher ( required by lidar range estimation)
- `rosbag play /catkin_ws/src/calibration files/ camera_info.bag -l` or `rosbag play /media/nvidia/37d85b64-6ae4-4913-beb8-75f3371b5284/camera_info.bag -l`
5. Run the lidar
`roslaunch velodyne_pointcloud VLP16_points.launch`
6. Run range estimation
`rosrun lidar_ranging lidar_ranging`

## Visualizing data
### Using rostopic to see what data is being published 
In ros, the messages and sensor data are published over 'topics'.  (you can think of it like a radio station)
To see what data is being published / subscribed to, run:
`rostopic list`

To see what is being published on a particular topic, 
run `rostopic echo TOPIC_NAME`

See this link for mor info https://wiki.ros.org/rostopic

### Using rviz to visualize the point cloud
- `rosrun rviz rviz -f velodyne`
- in the rviz window, click "Add" -> "By topic" -> /your_topic
- you can add images, point clouds, and other visual marker type messages
- when running lidar_ranging, add these topics:
1. /velodyne_points - the point cloud
2. /visualization_marker - visualization of the range estimation
3. /darknet_ros/image - the image with bounding boxes

## Recording and playing-back data
### Recording
To record, we use the rosbag tool https://wiki.ros.org/rosbag.  
The data we actually want to record form the sensor is published on these topics:
- /velodyne_points
- /videofile/image_raw

1. First, run `roscore`
2. Record a rosbag
- To record for a set duration, simple run: `rosbag record -O /PATH_TO_SSD/FILE_NAME --duration 2m /velodyne_points /videofile/image_raw`

- To record until you stop the script, run
`rosbag record -O /PATH_TO_SSD/FILE_NAME /velodyne_points /videofile/image_raw`

The rosbag recrod will record data to a .bag file in the specified location

### Play back
1. run `roscore`
2. play the saved bagfile: `rosbag play /PATH_TO_BAG/FILE_NAME.bag`
3. check that the data is being published: `rostopic list`  You should see /videofile/camera_raw and /velodyne_points
4. View the image stream
`rosrun image_transport_tutorial my_subscriber`
5. View the lidar point cloud
`rosrun rviz rviz -f velodyne`
6. You can run the object detection / lidar_ranging, and the should work just as if they were recieving live sensor data
- `roslaunch darknet_ros object_detect`
- `rosrun lidar_ranging lidar_ranging`

### how to do camera distortion calibration in ros
1. start ros `roscore`
2. run camera streaming on s32
- `ssh root@192.168.1.10`
- `./videoStream.elf`
3. camera streaming on jetson
- `rosrun image_transport_tutorial udp_publisher`
4. run camera calibration tool
`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera`

- size : (#columns-1) x (#rows - 1)
- square: size of tile in meters
- image:  topic to which images are published
- camera: base name for camera topic

5. It may not seem to be doing much, but if you move the checkerboard around, you will see the progress bar advance.  Keep moving the checkerboard around to different positions/orientations until it is complete