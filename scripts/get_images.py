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
        rospy.init_node('image_capture_node')
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path('insta360_ros_driver')
        self.img_path = os.path.join(self.pkg_dir, 'image_capture')
        if not os.path.exists(self.img_path):
            os.makedirs(self.img_path)
        self.topic = rospy.get_param('topic', '/back_camera_image/compressed')
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.sub_callback)
        self.img_counter = 0

    def sub_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            window_title = "Image Capture - Press 'q' to quit, 'SPACE' to capture"
            cv2.imshow(window_title, image)

            k = cv2.waitKey(1)
            if k == ord('q'): # 'q' Pressed
                print("Closing...")
                rospy.signal_shutdown("User requested shutdown")

            elif k%256 == 32: # SPACE Pressed
                img_name = "frame_{}.jpg".format(self.img_counter)
                cv2.imwrite(os.path.join(self.img_path, img_name), image)
                print("{} captured!".format(img_name))
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
