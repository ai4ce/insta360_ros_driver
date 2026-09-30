#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as exc:
    raise RuntimeError(
        "GStreamer Python bindings are required. Install python3-gi and "
        "gstreamer1.0-plugins-base."
    ) from exc


class RosGstreamerReceiver(Node):
    def __init__(self):
        super().__init__("ros_gstreamer_receiver")
        self.declare_parameter("topic", "/perspective/image/h264")
        self.declare_parameter(
            "pipeline",
            "appsrc name=ros_source is-live=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au,framerate=30/1 "
            "! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false",
        )
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("queue_depth", 10)

        self.topic = self.get_parameter("topic").value
        self.pipeline_description = self.get_parameter("pipeline").value
        self.fps = float(self.get_parameter("fps").value)
        queue_depth = int(self.get_parameter("queue_depth").value)
        if self.fps <= 0:
            raise ValueError("fps must be greater than zero")

        Gst.init(None)
        self.pipeline = Gst.parse_launch(self.pipeline_description)
        self.appsrc = self.pipeline.get_by_name("ros_source")
        if self.appsrc is None:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("The receiver pipeline must contain appsrc name=ros_source")

        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("block", False)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.frame_index = 0
        self.push_lock = threading.Lock()

        self.subscription = self.create_subscription(
            CompressedImage, self.topic, self.image_callback, queue_depth
        )
        self.get_logger().info(f"Receiving H.264 ROS stream on {self.topic}")

    def image_callback(self, message):
        if message.format.lower() not in ("h264", "h.264"):
            self.get_logger().warn(
                f"Expected CompressedImage format h264, received {message.format!r}",
                throttle_duration_sec=5.0,
            )
            return

        with self.push_lock:
            buffer = Gst.Buffer.new_allocate(None, len(message.data), None)
            buffer.fill(0, bytes(message.data))
            buffer.pts = self.frame_index * Gst.SECOND / self.fps
            buffer.dts = buffer.pts
            buffer.duration = Gst.SECOND / self.fps
            self.frame_index += 1
            result = self.appsrc.emit("push-buffer", buffer)

        if result != Gst.FlowReturn.OK:
            self.get_logger().error(f"GStreamer appsrc rejected frame: {result}")

    def destroy_node(self):
        if self.pipeline is not None:
            self.appsrc.emit("end-of-stream")
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RosGstreamerReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
