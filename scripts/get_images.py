#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge
import os
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage

class GetImages:    
    def __init__(self):
        rospy.init_node('undistort_node')
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path('insta360_ros_driver')
        self.sub = rospy.Subscriber('/front_camera_image/compressed', CompressedImage, self.sub_callback)

        self.img_counter = 0

    def sub_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("Image", image)

            k = cv2.waitKey(1)
            if k == 27:
                #ESC pressed
                print("Closing...")

            elif k%256 == 32:
                img_name = "frame_{}.jpg".format(self.img_counter)
                img_path = os.path.join(self.pkg_dir, 'distorted', img_name)
                cv2.imwrite(img_path, image)
                print("{} written!".format(img_name))
                self.img_counter += 1

        except Exception as e:
            print(e)
            return

if __name__ == '__main__':
    try:
        get_images = GetImages()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass