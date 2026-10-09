#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


class CppCalibrationClient(Node):
    def __init__(self):
        super().__init__('equirectangular_calibration_client')
        self.declare_parameter('target_node', 'dual_fisheye2equirectangular_node')
        self.declare_parameter('image_topic', '/equirectangular/image')
        self.target_node = self.get_parameter('target_node').value
        image_topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.pending = False
        self.values = self.read_parameters()
        self.get_client = self.create_client(GetParameters, f'/{self.target_node}/get_parameters')
        self.set_client = self.create_client(SetParameters, f'/{self.target_node}/set_parameters')
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.timer = self.create_timer(0.01, self.process_gui)
        self.window = 'C++ Equirectangular Calibration'
        self.controls = 'C++ Calibration Controls'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.controls, cv2.WINDOW_NORMAL)
        self.sliders = {}
        self.create_slider('CX Offset [-100,100]', self.values['cx_offset'] + 100, 200, 'cx')
        self.create_slider('CY Offset [-100,100]', self.values['cy_offset'] + 100, 200, 'cy')
        self.create_slider('Crop Size', self.values['crop_size'], 4096, 'crop')
        self.create_slider('TX [-0.5,0.5]', self.values['translation'][0] * 1000 + 500, 1000, 'tx')
        self.create_slider('TY [-0.5,0.5]', self.values['translation'][1] * 1000 + 500, 1000, 'ty')
        self.create_slider('TZ [-0.5,0.5]', self.values['translation'][2] * 1000 + 500, 1000, 'tz')
        self.create_slider('Roll [-180,180]', self.values['rotation_deg'][0] * 10 + 1800, 3600, 'roll')
        self.create_slider('Pitch [-180,180]', self.values['rotation_deg'][1] * 10 + 1800, 3600, 'pitch')
        self.create_slider('Yaw [-180,180]', self.values['rotation_deg'][2] * 10 + 1800, 3600, 'yaw')

    def read_parameters(self):
        values = {'cx_offset': 0.0, 'cy_offset': 0.0, 'crop_size': 0,
                  'translation': [0.0, 0.0, -0.105], 'rotation_deg': [-0.5, 0.0, 1.1]}
        client = self.create_client(GetParameters, f'/{self.target_node}/get_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            return values
        request = GetParameters.Request()
        request.names = ['cx_offset', 'cy_offset', 'crop_size', 'translation', 'rotation_deg']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result() and len(future.result().values) == 5:
            result = future.result().values
            values = {'cx_offset': result[0].double_value, 'cy_offset': result[1].double_value,
                      'crop_size': result[2].integer_value,
                      'translation': list(result[3].double_array_value),
                      'rotation_deg': list(result[4].double_array_value)}
        return values

    def create_slider(self, name, value, maximum, key):
        value = max(0, min(maximum, int(round(value))))
        self.sliders[key] = value
        cv2.createTrackbar(name, self.controls, value, maximum,
                           lambda current, slider_key=key: self.slider_changed(slider_key, current))

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(str(exc))

    def slider_changed(self, key, value):
        self.sliders[key] = value
        if self.pending or not self.set_client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter('cx_offset', Parameter.Type.DOUBLE, float(self.sliders['cx'] - 100)).to_parameter_msg(),
            Parameter('cy_offset', Parameter.Type.DOUBLE, float(self.sliders['cy'] - 100)).to_parameter_msg(),
            Parameter('crop_size', Parameter.Type.INTEGER, int(self.sliders['crop'])).to_parameter_msg(),
            Parameter('translation', Parameter.Type.DOUBLE_ARRAY,
                      [(self.sliders[key] - 500) / 1000.0 for key in ('tx', 'ty', 'tz')]).to_parameter_msg(),
            Parameter('rotation_deg', Parameter.Type.DOUBLE_ARRAY,
                      [(self.sliders[key] - 1800) / 10.0 for key in ('roll', 'pitch', 'yaw')]).to_parameter_msg(),
        ]
        self.pending = True
        future = self.set_client.call_async(request)
        future.add_done_callback(lambda _: setattr(self, 'pending', False))

    def process_gui(self):
        if self.latest_image is not None:
            cv2.imshow(self.window, self.latest_image)
        if cv2.waitKey(1) & 0xff == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CppCalibrationClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
