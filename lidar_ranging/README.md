## Setting up ROS on the jetson
### Initial Install
Follow the instructions here: https://wiki.ros.org/kinetic/Installation/Ubuntu

When you get to step 1.4, do the desktop-full install:

`sudo apt-get install ros-kinetic-desktop-full`
### Additional libraries to install
In general: `sudo apt-get install ros-kinetic-LIBRARY_NAME`
#### pcl
This is likely already installed with the desktop version of ros
https://wiki.ros.org/pcl
#### video_stream_opencv
https://wiki.ros.org/video_stream_opencv
(Optional) This can be used to visualize the camera stream from a video file.
If you do want to use this to view video, you will need to edit the file video_file.launch
To do this:
- `roscd video_stream_opencv`
- `cd launch`
- sudo gedit `video_file.launch`
- change the video_stream_provider to the absolute path of the video file
- change the camera_info_url to the lidar_ranging/ost.yaml, which contains camera metainfo
To run:
`rosrun video_stream_opencv video_file.launch`
## Compiling and building the project
1. Create the workspace
Follow the instructions here https://wiki.ros.org/catkin/Tutorials/create_a_workspace
2. Add `source ~/catkin_ws/devel/setup.bash` to the .bashrc file
3. Building all packges (under catkin_ws/src): `cd ~/catkin_ws` then  `catkin_make`
4. Building specific package: `catkin_make --pkg PACKAGE_NAME`
5. Clean command `catkin_make clean`

## Setting up the VLP-16
## How to record data
## How to play-back data
## Visualizing data with rviz
