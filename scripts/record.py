#!/usr/bin/env python3

import rospy
import rospkg
import os
import subprocess
import signal
import sys

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

    if bag_type == 'raw':
        command = f"rosbag record -o {raw_bag_folder}/ /insta_image_yuv"
        record_location = raw_bag_folder
    elif bag_type == 'compressed':
        command = f"rosbag record -o {compressed_bag_folder}/ /back_camera_image/compressed /front_camera_image/compressed"
        record_location = compressed_bag_folder
    elif bag_type == 'undistorted':
        command = f"rosbag record -o {undistorted_bag_folder}/ /back_camera_image/compressed /front_camera_image/compressed"
        record_location = undistorted_bag_folder
    else:
        rospy.logerr('Invalid bag_type parameter.')
        sys.exit(1)

    # Register the signal handler for SIGINT (Ctrl + C)
    signal.signal(signal.SIGINT, signal_handler)

    rospy.logwarn(f"Recording location: {record_location}")

    # Start the recording process
    process = subprocess.Popen(command, shell=True)

    # Keep the node alive
    rospy.spin()
