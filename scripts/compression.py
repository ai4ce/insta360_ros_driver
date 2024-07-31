#!/usr/bin/env python3
import rosbag
import rospy
import rospkg
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import os
from insta360_ros_driver.tools import *

if __name__ == '__main__':
    rospy.init_node("compession_node")
    bridge = CvBridge()

    topic_name = '/insta_image_yuv'
    default_dir = rospkg.RosPack().get_path('insta360_ros_driver')
    default_dir = os.path.join(default_dir, 'bag')
    bag_dir = rospy.get_param('bag_dir', default_dir)
    #Check if folder with raw bag files exist
    if not os.path.exists(os.path.join(bag_dir, 'raw')):
        rospy.logerr("No raw data found in the specified directory")
        exit()
    
    bag_filenames = os.listdir(os.path.join(bag_dir, 'raw'))
    bag_filenames = [f for f in bag_filenames if f.endswith('.bag') and not f.startswith('.')]
    bag_filenames.sort()
    bag_filenames.sort(key=len)
    print(bag_filenames)

    bag_paths = [os.path.join(bag_dir, 'raw', bag_filename) for bag_filename in bag_filenames]

    if not os.path.exists(os.path.join(bag_dir, 'compressed')):
        os.makedirs(os.path.join(bag_dir, 'compressed'))
    
    outbag_filenames = [filename.split('.')[0] + '_compressed.bag' for filename in bag_filenames]
    outbag_paths = [os.path.join(bag_dir, 'compressed', outbag_filename) for outbag_filename in outbag_filenames]

    for i in tqdm(range(len(bag_paths))):
        try:
            with rosbag.Bag(bag_paths[i], 'r') as bag:
                with rosbag.Bag(outbag_paths[i], 'w') as outbag:
                    for topic, msg, t in tqdm(bag.read_messages()):
                        if topic == topic_name:
                            image = bridge.imgmsg_to_cv2(msg,   desired_encoding='passthrough')
                            # Convert the YUV image to BGR format
                            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
                            front_image, back_image = split_image(bgr_image)

                            #Compress image to JPG and create msg
                            front_compressed_msg = compress_image_to_msg(front_image, t)
                            back_compressed_msg = compress_image_to_msg(back_image, t)

                            # Publish the compressed images
                            outbag.write('/front_camera_image/compressed', front_compressed_msg, t)
                            outbag.write('/back_camera_image/compressed', back_compressed_msg, t)
                        else:
                            outbag.write(topic, msg, t)   
        except Exception as e:
            print(e)
            continue