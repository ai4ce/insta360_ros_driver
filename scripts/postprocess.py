#!/usr/bin/env python3
from Curb2Door.dependencies.insta360_ros_driver.scripts.live_processing import D, K
import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
from tqdm import tqdm
import os
import rospkg

def split_image(image):
    height, width = image.shape[:2]
    mid_point = width // 2
    front_image = image[:, :mid_point]
    back_image = image[:, mid_point:]

    return front_image, back_image

def undistort_image(image, K, D):
    h, w = image.shape[:2]
    new_K = K.copy()
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_32FC1)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted_img

def compress_image_to_msg(image, timestamp):
    _, buffer = cv2.imencode('.jpg', image)
    image_msg = CompressedImage()
    image_msg.header.stamp = timestamp
    image_msg.format = 'jpeg'
    image_msg.data = buffer.tobytes()
    return image_msg

if __name__ == '__main__':
    rospy.init_node('processing_node')
    bridge = CvBridge()
    rospack = rospkg.RosPack()
    
    topic_name = rospy.get_param('topic_name')
    is_yuv = rospy.get_param('is_yuv')
    is_compressed = rospy.get_param('is_compressed')
    rotate = rospy.get_param('rotate', False)
    K = np.array(rospy.get_param('K'))
    D = np.array(rospy.get_param('D'))

    pkg_dir = rospack.get_path('insta360_ros_driver')
    bag_filenames = os.listdir(os.path.join(pkg_dir, 'bag_raw'))
    bag_filenames = [f for f in bag_filenames if f.endswith('.bag') and not f.startswith('.')]
    bag_filenames.sort()
    bag_filenames.sort(key=len)
    print(bag_filenames)
    bag_paths = [os.path.join(pkg_dir, 'bag_raw', bag_filename) for bag_filename in bag_filenames]

    outbag_filenames = [filename.split('.')[0] + '_processed.bag' for filename in bag_filenames]
    outbag_paths = [os.path.join(pkg_dir, 'bag_processed', outbag_filename) for outbag_filename in outbag_filenames]

    for i in tqdm(range(len(bag_paths))):
        try:
            with rosbag.Bag(bag_paths[i], 'r') as bag:
                with rosbag.Bag(outbag_paths[i], 'w') as outbag:
                    for topic, msg, t in tqdm(bag.read_messages()):
                        if topic == topic_name:
                            if is_compressed:
                                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                            else:
                                image = bridge.imgmsg_to_cv2(msg,   desired_encoding='passthrough')
                            # Convert the YUV image to BGR format
                            if is_yuv:
                                bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
                            else:
                                bgr_image = image

                            front_image, back_image = split_image(bgr_image)

                            if(rotate):
                                front_image = cv2.rotate(front_image, cv2.ROTATE_90_CLOCKWISE)
                                back_image= cv2.rotate(back_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

                            #Undistort images
                            front_image = undistort_image(front_image, K, D)
                            back_image = undistort_image(back_image, K, D)

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