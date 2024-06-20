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

#X2 Parameters
# mtx = np.array([[795.02, 0.0, 873.05],
#              [0.0, 795.3, 836.3],
#              [0.0, 0.0, 1.0]])

# dist = np.array([-0.13020864, -0.0165325 ,  0.01075378,  0.00457717,  0.00664401])

undistort=True

K = np.array([[568.6654432377414, 0.0, 955.6883006362993], 
     [0.0, 569.6599355654018, 961.1454179184642], 
     [0.0, 0.0, 1.0]])

D = np.array([[0.03864975643881084], [0.00799517789050036], [-0.008943738947933197], [0.0006243938707961845]])


#Air Parameters
# K = np.array([[473, 0.0, 752], 
#               [0.0, 473, 752], 
#               [0.0, 0.0, 1.0]])
# D = np.array([-0.020286852335397013, -0.007, 0.005, -0.003])

class LiveProcessing():
    def __init__(self):
        rospy.init_node('compression_node')
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('insta_image_yuv', Image, self.processing)
        self.front_image_pub = rospy.Publisher('front_camera_image/compressed', CompressedImage, queue_size=10)
        self.back_image_pub = rospy.Publisher('back_camera_image/compressed', CompressedImage, queue_size=10)

    def processing(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]

            # Rotate front image 90 degrees clockwise
            front_image = cv2.rotate(front_image, cv2.ROTATE_90_CLOCKWISE)

            # Rotate back image 90 degrees counterclockwise
            back_image = cv2.rotate(back_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            if(undistort):
                front_image = undistort_image(front_image, K, D)
                back_image = undistort_image(back_image, K, D)

                # front_image = undistort_image_classic(front_image, mtx, dist)
                # back_image = undistort_image_classic(back_image, mtx, dist)

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