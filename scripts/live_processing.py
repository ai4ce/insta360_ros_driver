#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

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

class LiveProcessing():
    def __init__(self):
        rospy.init_node('live_processing_node')
        self.bridge = CvBridge()
        
        self.topic_name = rospy.get_param("topic_name", '/insta_image_yuv')
        self.is_yuv = rospy.get_param("is_yuv", True)
        self.is_compressed = rospy.get_param("is_compressed", False)
        self.undistort = rospy.get_param("/undistort", False)
        self.K = np.asarray(rospy.get_param("K"))
        self.D = np.asarray(rospy.get_param("D"))

        self.image_sub = rospy.Subscriber(self.topic_name, Image, self.processing)
        self.front_image_pub = rospy.Publisher('front_camera_image/compressed', CompressedImage, queue_size=10)
        self.back_image_pub = rospy.Publisher('back_camera_image/compressed', CompressedImage, queue_size=10)


    def processing(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)

            print(f"Image Size: {bgr_image.shape[:2]}")

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]

            # Rotate front image 90 degrees clockwise
            front_image = cv2.rotate(front_image, cv2.ROTATE_90_CLOCKWISE)

            # Rotate back image 90 degrees counterclockwise
            back_image = cv2.rotate(back_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            if(self.undistort):
                front_image = undistort_image(front_image, self.K, self.D)
                back_image = undistort_image(back_image, self.K, self.D)

            front_compressed_msg = compress_image_to_msg(front_image, msg.header.stamp)
            back_compressed_msg = compress_image_to_msg(back_image, msg.header.stamp)

            # Publish the compressed images
            self.front_image_pub.publish(front_compressed_msg)
            self.back_image_pub.publish(back_compressed_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s", e)
        except Exception as e:
            rospy.logerr("Failed to process image: %s", e)

if __name__ == '__main__':
    try:
        lp = LiveProcessing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass