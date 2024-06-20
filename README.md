# insta360_ros_driver

A ROS driver for the Insta360 X2 camera. This driver is tested on Ubuntu 20.04 with ROS Noetic. 

## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to installed as follows: 
- add the header <code>camera</code> and <code>stream</code> header files inside the <code>include</code>
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Usage
In order to access the camera feed, administrator privileges are required. Furthermore, the camera natively records in YUV format. The conversion to BGR/RGB is CPU-intensive, so this is done in post processing. To use this driver, use the following commands to record data and then process them.

### Raw Data Recording
Use the following to start recording. Recordings are saved as bag files under the <code>bag_raw</code> directory.
```
sudo -s
source devel/setup.bash
roslaunch insta360_ros_driver record.launch
```
The raw YUV image appears as shown below
![yuv.png](docs/yuv.png)

### Data Processing
After recording, the following commands process all the commands in the <code>bag_raw</code> directory and save them in the <code>bag_processed</code> directory. The processing involves the folowing steps:

- Conversion from YUV to BGR
- Image splitting and rotation
- Image undistortion
- Compression into JPEG
```
roslaunch insta360_ros_driver process.launch
```
The processed image appears as shown below
![yuv.png](docs/bgr.png)

### Live Processing and Preview (Worse Performance)
It is also possible to view the output of the camera live. This is useful since the raw bag files may be large in size, so doing the processing live can save on space. However, there is a higher chance of image artifacts appearing since this is computationally expensive.
```
roslaunch insta360_ros_driver live_preview.launch
```
Undistortion can also be added to the live_preview, at the cost of additional computational load. 
![undistort.png](docs/undistort.png)