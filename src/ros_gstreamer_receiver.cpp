#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <cstdio>

class RosGstreamerReceiver : public rclcpp::Node {
public:
    RosGstreamerReceiver() : Node("ros_gstreamer_receiver") {
        declare_parameter("topic", "/perspective/image/h264");
        declare_parameter("fps", 30.0);
        declare_parameter("pipeline", std::string("appsrc name=source is-live=true format=time ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false"));
        topic_ = get_parameter("topic").as_string();
        fps_ = get_parameter("fps").as_double();
        gst_init(nullptr, nullptr);
        GError *error = nullptr;
        pipeline_ = gst_parse_launch(get_parameter("pipeline").as_string().c_str(), &error);
        if (error || !pipeline_) throw std::runtime_error(error ? error->message : "GStreamer pipeline failed");
        source_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
        if (!source_) throw std::runtime_error("receiver pipeline needs appsrc name=source");
        gst_app_src_set_stream_type(source_, GST_APP_STREAM_TYPE_STREAM);
        g_object_set(G_OBJECT(source_), "format", GST_FORMAT_TIME, nullptr);
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(topic_, rclcpp::SensorDataQoS(), std::bind(&RosGstreamerReceiver::callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Receiving %s", topic_.c_str());
    }
    ~RosGstreamerReceiver() override {
        if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (source_) gst_object_unref(source_);
        if (pipeline_) gst_object_unref(pipeline_);
    }
private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->format != "h264") return;
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(frame_index_++, GST_SECOND, fps_);
        if (gst_app_src_push_buffer(source_, buffer) != GST_FLOW_OK) RCLCPP_WARN(get_logger(), "GStreamer rejected frame");
    }
    std::string topic_;
    double fps_;
    guint64 frame_index_ = 0;
    GstElement *pipeline_ = nullptr;
    GstAppSrc *source_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<RosGstreamerReceiver>()); }
    catch (const std::exception &exc) { fprintf(stderr, "%s\n", exc.what()); }
    rclcpp::shutdown();
    return 0;
}
