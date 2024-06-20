#!/usr/bin/env python3
import rosbag
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
from tqdm import tqdm
import os
import rospkg


# topic_name = '/insta_image_yuv'
# is_yuv = True
# is_compressed = False

topic_name = '/camera360/image_raw/compressed'
is_yuv = False
is_compressed = True

bridge = CvBridge()
rospack = rospkg.RosPack()

#Air Intrinsics:
K = np.array([[473, 0.0, 752], [0.0, 473, 752], [0.0, 0.0, 1.0]])
D = np.array([-0.020286852335397013, -0.007, 0.005, -0.003])

#X2 Intrinsics:
# K = np.array([[568.6654432377414, 0.0, 955.6883006362993], 
#      [0.0, 569.6599355654018, 961.1454179184642], 
#      [0.0, 0.0, 1.0]])

# D = np.array([[0.03864975643881084], [0.00799517789050036], [-0.008943738947933197], [0.0006243938707961845]])

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

def undistort_image_classic(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def compress_image_to_msg(image, timestamp):
    _, buffer = cv2.imencode('.jpg', image)
    image_msg = CompressedImage()
    image_msg.header.stamp = timestamp
    image_msg.format = 'jpeg'
    image_msg.data = buffer.tobytes()
    return image_msg

if __name__ == '__main__':
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

                        # Rotate front image 90 degrees clockwise
                        # front_image = cv2.rotate(front_image, cv2.ROTATE_90_CLOCKWISE)
                        # back_image= cv2.rotate(back_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

                        #Undistort images
                        front_image = undistort_image(front_image, K, D)
                        back_image = undistort_image(back_image, K, D)

                        # front_image = undistort_image_classic(front_image, mtx, dist)
                        # back_image = undistort_image_classic(back_image, mtx, dist)

                        #Compress image to JPG and create msg
                        front_compressed_msg = compress_image_to_msg(front_image, t)
                        back_compressed_msg = compress_image_to_msg(back_image, t)

                        # Publish the compressed images
                        outbag.write('/front_camera_image/compressed', front_compressed_msg, t)
                        outbag.write('/back_camera_image/compressed', back_compressed_msg, t)
                    else:
                        outbag.write(topic, msg, t)