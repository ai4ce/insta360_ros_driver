#!/usr/bin/env python3

import rospy
import rospkg
import os
import subprocess
import signal
import sys
import datetime

# Function to handle SIGINT (Ctrl + C)
def signal_handler(sig, frame):
    global process
    rospy.logwarn('Stopping recording...')
    process.terminate()
    process.wait()
    rospy.logwarn('Recording stopped.')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('record_node')
    bag_type = rospy.get_param('bag_type', 'compressed')

    default_dir = rospkg.RosPack().get_path('insta360_ros_driver')
    default_dir = os.path.join(default_dir, 'bag')
    raw_bag_folder = rospy.get_param('raw_bag_folder', os.path.join(default_dir, 'raw'))
    compressed_bag_folder = rospy.get_param('compressed_bag_folder', os.path.join(default_dir, 'compressed'))
    undistorted_bag_folder = rospy.get_param('undistorted_bag_folder', os.path.join(default_dir, 'undistorted'))

    time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

    if bag_type == 'raw':
        filename = f"{raw_bag_folder}/{time}_raw.bag"
        command = f"rosbag record -O {filename} /insta_image_yuv"
        record_location = raw_bag_folder
    elif bag_type == 'compressed':
        filename = f"{compressed_bag_folder}/{time}_compressed.bag"
        command = f"rosbag record -O {filename} /back_camera_image/compressed /front_camera_image/compressed"
        record_location = compressed_bag_folder
    elif bag_type == 'undistorted':
        filename = f"{undistorted_bag_folder}/{time}_undistorted.bag"
        command = f"rosbag record -O {filename} /back_camera_image/compressed /front_camera_image/compressed"
        record_location = undistorted_bag_folder
    else:
        rospy.logerr('Invalid bag_type parameter.')
        sys.exit(1)

    # Register the signal handler for SIGINT (Ctrl + C)
    signal.signal(signal.SIGINT, signal_handler)

    rospy.logwarn(f"Recording to: {filename}")

    # Start the recording process
    process = subprocess.Popen(command, shell=True)

    # Keep the node alive
    rospy.spin()
