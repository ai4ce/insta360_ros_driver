#!/usr/bin/env python3

import rospy
import os
import rospkg

def verify_directories():
    default_dir = rospkg.RosPack().get_path('insta360_ros_driver')
    default_dir = os.path.join(default_dir, 'bag')

    raw_bag_folder = rospy.get_param('raw_bag_folder', os.path.join(default_dir, 'raw'))
    compressed_bag_folder = rospy.get_param('compressed_bag_folder', os.path.join(default_dir, 'compressed'))
    undistorted_bag_folder = rospy.get_param('undistorted_bag_folder', os.path.join(default_dir, 'undistorted'))
    
    folders = [raw_bag_folder, compressed_bag_folder, undistorted_bag_folder]

    for folder in folders:
        if not os.path.exists(folder):
            os.makedirs(folder)
            rospy.loginfo(f"Created folder {folder}")

if __name__ == '__main__':
    verify_directories()