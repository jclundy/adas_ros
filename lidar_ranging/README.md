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

## How to record data

## How to play-back data
## Visualizing data
### seeing what data is being published
### using rviz to visualize the point cloud
