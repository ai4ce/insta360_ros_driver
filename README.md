# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 20.04 with ROS Noetic. The driver has also been verified on the Insta360 X2 and X3 cameras.

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

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
rosrun insta360_ros_driver setup.sh
```
This create a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.
![setup](docs/setup.png)

## Usage
This driver directly publishes the video feed in YUV format, since that is the camera's native setting. Alongside this, the driver also publishes the camera feed as standard BGR images to the <code>/front_camera_image/compressed</code> and <code>/back_camera_image/compressed</code> topics. Note that the compressed images have some amount of latency (~50 ms) compared to the raw output. 

### Camera Bringup
The camera can be brought up with the following launch file
```
roslaunch insta360_ros_driver bringup.launch
```
This publishes the raw yuv and compressed images as the following topics
- /insta_image_yuv
- /front_camera_image/compressed
- /back_camera_image/compressed

The launch file has the following optional arguments:
- compress (default="true")

Whether to publish the compressed front and back images. When set to false, only the raw YUV image is published

- undistort (default="false")

When set to true, it will use the specified intrinsic matrix, found in <code>config/intrinsic.yaml</code> to undistort the images in real-time. Note that this process can reduce the frame rate and/or increase latency. It is therefore recommended to keep it as 'false' and perform undistortion in post-processing, as will be shown in the next section.

- debug (default="false")

When set to true, additional diagnostic windows are created. These show the publishing frequency and image size. 

### Recording to a Bag File
While the camera driver is active (via the bringup launch file above), data can be recorded to a bag file using
```
roslaunch insta360_ros_driver bag_record.launch
```
By default, it will save the <code>/front_camera_image/compressed</code> and <code>/back_camera_image/compressed</code> topics to the <code>bag/compressed</code> folder.

Since this repository also contains tools for image compression and undistortion, the bag files, by default, are structured as follows:

```
insta360_ros_driver/
|--bag/
   |--raw/
   |--compressed/
   |--undistorted/
```

The launch file has the following optional arguments.
- bag_dir (default="insta360_ros_driver/bag")

This is the higher-level "bag" folder that contains the "raw", "compressed", and "undistorted" subdirectories. 

- bag_type (default="compressed", options="\["raw", "compressed", "undistorted"\])

This specifies which subdirectory to save the bag files to. 

When set to "raw", it will record a bag file into the <mode>bag/raw</mode> folder with only the <code>/insta_image_yuv</code> topic. 

When set to "compressed", it will record a bag file into the <mode>bag/compressed</mode> folder with only the <code>/front_camera_image</code> and <code>/back_camera_image</code> topics. 

When set to "undistorted", the recorded topics are the same as when set to "compressed" but the file will be saved to the <code>bag/undistorted</code> folder.

Note that the RAW YUV image appears as follows:

![YUV](docs/yuv.png)

### Post-Processing: Image Compression and Splitting
Given a bag file containing raw YUV image data, the images can be split to front and back and also compressed using the following launch file.
```
roslaunch insta360_ros_driver bag_compress.launch
```
The launch file has one optional argument:
- bag_dir (default="insta360_ros_driver/bag")

This is the higher-level "bag" folder that contains the "raw", "compressed", and "undistorted" subdirectories. By default, then, it will look for all raw bag files in the <code>insta360_ros_driver/bag/raw</code> directory.

This launch file will output bag files with the <code>/front_camera_image/compressed</code> and <code>back_camera_image/compressed</code> topics in the <code>compressed</code> bag subdirectory.

After compression and splitting, the images appear as follows:
![BGR](docs/bgr.png)

### Post-Processing: Image Undistortion
Given a bag file containing compressed front and back images, these images can be undistorted using the following launch file.
```
roslaunch insta360_ros_driver bag_undistort.launch
```
The launch file has one optional argument:
- bag_dir (default="insta360_ros_driver/bag")

This is the higher-level "bag" folder that contains the "raw", "compressed", and "undistorted" subdirectories. By default, then, it will look for all compressed bag files in the <code>insta360_ros_driver/bag/compressed</code> directory.

This launch file will output bag files with the undistorted images in the <code>undistorted</code>

After undistortion, the images appear as follows:
![undistorted](docs/undistort.png)

### Bag Preview
The compressed and undistorted bag files can be previewed using RViz via the following launch file.
```
roslaunch insta360_ros_driver bag_preview.launch
```

The launch file has the following arguments:
- bag_dir (default="insta360_ros_driver/bag")

This is the higher-level "bag" folder that contains the "raw", "compressed", and "undistorted" subdirectories.

- bag_type (default="compressed", options=\["compressed", "undistorted"\])

This tells the launch file which subdirectory to look for the bag file.

- bag_file (default="record.bag")

This is the filename of the bag file that will be previewed. Please make sure to change this accordingly. 

When launched, an RViZ window will be created showing both front and back images, as was previously demonstrated in earlier sections of the documentation