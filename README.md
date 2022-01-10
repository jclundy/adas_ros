See design reports and presentations for lidar-camera sensor fusion on google drive: [UWAFT ADAS Developer](https://drive.google.com/drive/folders/1hY_oJtX0f3rVMeduFmjDEalWaxDfT7E8?usp=sharing)

Original README below:

# adas_ros
Where it all comes together. This is the main program that will run the ADAS system in the car on the Jetson TX2.

## Data Flow
Camera video is streamed over UDP socket from the NXP S32V to the Jetson on Ethernet. LiDAR data from the Velodyne VLP-16 is also streaming to the Jetson through Ethernet. The Jetson then processes the video by running it through TinyYOLO to identify vehicles. The bounding boxes found by YOLO are then queried against the LiDAR data to get the average distance in that bounding box to our vehicle. The bounding boxes is also tracked so the same vehicle does not get identified multiple times.


## To-Do
- [x] Feed video streamed over UDP to YOLO
- [x] Pass bounding boxes to LiDAR for distance estimation
- [x] Bounding Box positional tracking
- [x] Record output video
- [x] Reduce YOLO catagories
- [ ] Driver Feedback (Lift off accel and brake thresholds)
- [ ] Data logging on CAN
- [ ] Multi object range estimation/ system test
- [ ] Auto boot (S32V234, fix run_live_detect_range.sh)
- [ ] Solder jumper cable for S32V234
- [ ] Solder CAN board to Jetson
- [ ] Communication between RPi and Jetson
- [ ] brake light detection (assume first frame of detection is brake not applied)


## Building YOLO/Darknet
When building YOLO fresh, ensure the flags in the `Makefile` indicate that CUDA and OPENCV are to be built.

## Usage
To run program using live data  
`./run_live_detect_range.sh`  
To run program using saved ros bag files (ensure SSD is mounted and files are present in the required location)  
`./run_lidar_camera.sh`

## Building individual ros packages
`catkin_make --pkg <package-name>`  
example: CAN  
`catkin_make --pkg can`
