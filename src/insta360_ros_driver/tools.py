#!/usr/bin/env python3
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