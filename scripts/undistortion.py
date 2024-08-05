#!/usr/bin/env python3
import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from tqdm import tqdm
import os
import rospkg
from insta360_ros_driver.tools import undistort_image, compress_image_to_msg
from insta360_ros_driver.directory_verification import verify_directories

if __name__ == '__main__':
    rospy.init_node("undistortion_node")
    bridge = CvBridge()

    topic_names = ['/front_camera_image/compressed', '/back_camera_image/compressed']
    K = np.asarray(rospy.get_param("K", np.eye(3,3)))
    D = np.asarray(rospy.get_param("D", np.zeros(4)))

    verify_directories()
    default_dir = rospkg.RosPack().get_path('insta360_ros_driver')
    default_dir = os.path.join(default_dir, 'bag')
    compressed_bag_folder = rospy.get_param('compressed_bag_folder', os.path.join(default_dir, 'compressed'))
    undistorted_bag_folder = rospy.get_param('undistorted_bag_folder', os.path.join(default_dir, 'undistorted'))

    bag_filenames = os.listdir(compressed_bag_folder)
    bag_filenames = [f for f in bag_filenames if f.endswith('.bag') and not f.startswith('.')]
    bag_filenames.sort()
    bag_filenames.sort(key=len)
    print(bag_filenames)

    bag_paths = [os.path.join(compressed_bag_folder, bag_filename) for bag_filename in bag_filenames]
    
    outbag_filenames = [filename.split('.')[0] + '_undistorted.bag' for filename in bag_filenames]
    outbag_paths = [os.path.join(undistorted_bag_folder, outbag_filename) for outbag_filename in outbag_filenames]

    for i in tqdm(range(len(bag_paths))):
        try:
            with rosbag.Bag(bag_paths[i], 'r') as bag:
                with rosbag.Bag(outbag_paths[i], 'w') as outbag:
                    for topic, msg, t in tqdm(bag.read_messages()):
                        if topic in topic_names:
                            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                            image = undistort_image(image, K, D)
                            image_msg = compress_image_to_msg(image, t)
                            outbag.write(topic, image_msg, t)
                        else:
                            outbag.write(topic, msg, t)
        except Exception as e:
            print(e)
            continue

