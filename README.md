# CHANGES OF THIS FORK
- Dynamic parameters change works correctly
- Equirectangular node is now more efficient
- Perspective node have been added. You can control fov through parameters and camera orientation by publishing to: /&#8288;camera_orientation/&#8288;quaternion

# Insta360 ROS2 Jazzy Driver with Pixi Environment

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/avasalya/insta360_ros_driver/blob/65e9cef35152a7af368e8b1f063ce85e8780d2a1/src/main.cpp#L130) to edit the resolution.

```cpp
param.video_resolution = ins_camera::VideoResolution::RES_1920_960P30; //Change this line to edit the resolution
```

# Installation

To use this driver, you need the latest Insta360 SDK (post-April 23, 2025), which can be requested via their [official website](https://insta360.com/sdk/home). For additional instructions, refer to this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

> ⚠️ **Note:** Do not manually clone or build this submodule directly. This package is managed within a `pixi` ecosystem to avoid environment conflicts. Also Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**


Please follow the installation guide in the **[parent repository](https://github.com/avasalya/pixi_insta360_ros2_jazzy_driver)**.

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/avasalya/pixi_insta360_ros2_jazzy_driver 
cd pixi_insta360_ros2_jazzy_driver
```

**Add dependencies:** 
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

**Build:** From the parent repository root, run:
```bash
# Setup: install dependencies and build
pixi run -e jazzy360 setup
```

# Setup Insta360 Camera 

**make sure the camera is set to dual-lens (360°) mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:

```bash
cd ~/pixi_insta360_ros2_jazzy_driver/src/insta360_ros_driver
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

The launch file has the following optional arguments:
- equirectangular (default="false")

This publishes equirectangular images. You can configure these parameters in `config/equirectangular.yaml`.
![equirectangular](docs/equirectangular.png)

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
