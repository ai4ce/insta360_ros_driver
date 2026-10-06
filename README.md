# CHANGES OF THIS FORK
- Dynamic parameters change works correctly
- Equirectangular node is now more efficient
- Perspective node have been added. You can control fov through parameters and camera orientation by publishing to: /&#8288;camera_orientation/&#8288;quaternion

# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/ai4ce/insta360_ros_driver/blob/79588d9e0e9d029c3371d4095ea718daaf1e06fb/src/main.cpp#L126) to edit the resolution.

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

```
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Before continuing, **make sure the camera is set to dual-lens mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Usage
The camera provides images natively in H264 compressed image format. We have a decoder node that 

### Camera Bringup
The camera can be brought up with the following launch file
```
ros2 launch insta360_ros_driver bringup.launch.xml
```
![bringup](docs/bringup_rqt.png)

A dual fisheye image will be published.

![dual_fisheye](docs/dual_fisheye.png)

#### Published Topics
- /dual_fisheye/image
- /dual_fisheye/image/compressed
- /equirectangular/image
- /imu/data
- /imu/data_raw

#### MediaSDK Realtime Stitching

The provided MediaSDK package contains `ins::RealTimeStitcher`, which accepts the
camera's H.264/H.265 access units and IMU data and returns stitched frames. Install
the supplied Debian package first.The package is contained in before mentioned SDK that you have to apply for [Insta360 website](https://www.insta360.com/sdk/home):

```bash
sudo apt install ./libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.deb
```
To compile against a non-system SDK prefix:

```bash
colcon build --symlink-install --cmake-args \
  -DINSTA360_MEDIA_SDK_ROOT=/path/to/MediaSDK/prefix
```
Then run the camera and the MediaSDK node. The node consumes the topics produced by
`main.cpp` and publishes a stitched equirectangular image:

```bash
ros2 run insta360_ros_driver insta360_ros_driver
ros2 run insta360_ros_driver media_sdk_stitcher --ros-args \
  --params-file config/media_sdk_stitcher.yaml
```

The default node uses `dynamicstitch`; `template`, `optflow`, and `aistitch` are
also accepted. Camera-specific values are read automatically from
`CameraSDK::GetPreviewParam()` by `main.cpp` and published on the reliable,
transient-local `/insta360/camera_preview_info` topic. The MediaSDK runtime model files
are installed by the Debian package. For CUDA/model failures, try
`stitch_type: template` first.



The launch file has the following optional arguments:
- `webcam_equirectangular` is a separate node for cameras switched to webcam/UVC mode.

When the camera exposes a stitched 2:1 webcam mode, list the available V4L2
devices and formats:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

Run the node with the matching device:

```bash
ros2 run insta360_ros_driver webcam_equirectangular --ros-args \
  --params-file config/webcam_equirectangular.yaml
```

It requests `1920x960`, validates the returned frame is 2:1, converts it to
`rgb8`, and publishes `/equirectangular/image` as a raw ROS 2 image. Webcam/UVC
mode is controlled by Linux and the camera firmware, not by CameraSDK.

- equirectangular (default="false")

This publishes equirectangular images. You can configure these parameters in `config/equirectangular.yaml`.
![equirectangular](docs/equirectangular.png)

#### GStreamer Nodes

The conversion nodes publish raw ROS images only. Encoding is handled by a separate node:

```bash
ros2 run insta360_ros_driver gstreamer_encoder --ros-args \
  -p input_topic:=/equirectangular/image \
  -p transport:=ros \
  -p output_topic:=/equirectangular/image/h264
```

For direct RTP/UDP output, use `transport:=udp` and set `pipeline`:

```bash
ros2 run insta360_ros_driver gstreamer_encoder --ros-args \
  -p input_topic:=/equirectangular/image \
  -p transport:=udp \
  -p pipeline:="appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=4000 ! rtph264pay pt=96 ! udpsink host=192.168.1.50 port=5000"
```

To decode an H.264 ROS topic back to a raw image topic:

```bash
ros2 run insta360_ros_driver gstreamer_decoder --ros-args \
  -p compressed_topic:=/equirectangular/image/h264 \
  -p image_topic:=/equirectangular/image/decoded
```

`dual_fisheye2equirectangular_cpp` is the supported ROS stitcher for the live dual-fisheye topic. The checked-in Insta360 SDK exposes camera-side `EnableInCameraStitching`, but does not expose a frame-level MediaSDK stitching API, so the package does not claim to perform MediaSDK stitching in a ROS node.

The old FFmpeg decoder is explicitly named `ffmpeg_decoder`:

```bash
ros2 run insta360_ros_driver ffmpeg_decoder
```

Install GStreamer development/runtime packages:

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

Both machines must be able to discover each other through ROS 2 DDS and use the same `ROS_DOMAIN_ID`.

For direct UDP mode, receive the stream with:

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
```

- imu_filter (default="true")

This uses the [imu_filter_madgwick](https://wiki.ros.org/imu_filter_madgwick) package to approximate orientation from the IMU. Note that by default, we publish `/imu/data_raw` which only contains linear acceleration and angular velocity. The madgwick filter uses this information to publish orientation to `/imu/data`. You can configure the filter in `config/imu_filter.yaml`. 

![IMU](https://github.com/user-attachments/assets/02b50cad-8415-4dde-9014-9ab3a4d415b9)

## Equirectangular Calibration
You can adjust the extrinsic parameters used to improve the equirectangular image. 
```
# Run the camera driver
ros2 run insta360_ros_driver insta360_ros_driver
# Activate image decoding
ros2 run insta360_ros_driver decoder
# Run the equirectangular node in calibration mode
ros2 run insta360_ros_driver equirectangular.py --calibrate
```
This will open an app to adjust the extrinsics. You can press 's' to get the parameters in YAML format. **Note that you need to press 'a' to update the image preview after changing the intrinsics with the GUI**
![Equirectangular Calibration](docs/calibration.png)

Pressing 's' will return the parameters via the terminal. You can copy paste this onto the configuration file as needed. By default, the launch file reads this from `config/equirectangular.yaml`

For the C++ projection node, use the separate client. It displays `/equirectangular/image` and updates the C++ node parameters through ROS:

```bash
ros2 run insta360_ros_driver calibrate_cpp.py \
  --ros-args -p target_node:=dual_fisheye2equirectangular_node
```

```
==================================================
CALIBRATION PARAMETERS (YAML FORMAT)
==================================================
equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 960
    translation: [0.0, 0.0, -0.105]
    rotation_deg: [-0.5, 0.0, 1.1]
    gpu: True
    out_width: 1920
    out_height: 960
==================================================
```

Note that decode.py will most likely drop frames depending on your system. If you do not care about live processing, you can simply record the `/dual_fisheye/image/compressed` topic and decompress it later after recording.
```
ros2 bag record /dual_fisheye/image /imu/data_raw
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)
