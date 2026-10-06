#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <stdexcept>

class GstreamerDecoderNode : public rclcpp::Node {
public:
    GstreamerDecoderNode() : Node("gstreamer_decoder") {
        declare_parameter("compressed_topic", "/dual_fisheye/image/compressed");
        declare_parameter("image_topic", "/dual_fisheye/image");
        declare_parameter("pipeline", std::string("appsrc name=source is-live=true format=time ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink sync=false"));
        topic_ = get_parameter("compressed_topic").as_string();
        image_topic_ = get_parameter("image_topic").as_string();
        pipeline_description_ = get_parameter("pipeline").as_string();
        gst_init(nullptr, nullptr);
        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
        if (error || !pipeline_) throw std::runtime_error(error ? error->message : "GStreamer pipeline failed");
        source_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
        sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));
        if (!source_ || !sink_) throw std::runtime_error("pipeline needs appsrc name=source and appsink name=sink");
        gst_app_src_set_stream_type(source_, GST_APP_STREAM_TYPE_STREAM);
        g_object_set(G_OBJECT(source_), "format", GST_FORMAT_TIME, nullptr);
        gst_app_sink_set_drop(sink_, true);
        gst_app_sink_set_max_buffers(sink_, 1);
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
        subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(topic_, rclcpp::SensorDataQoS(), std::bind(&GstreamerDecoderNode::callback, this, std::placeholders::_1));
    }
    ~GstreamerDecoderNode() override {
        if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (source_) gst_object_unref(source_);
        if (sink_) gst_object_unref(sink_);
        if (pipeline_) gst_object_unref(pipeline_);
    }
private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->format != "h264") return;
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        if (gst_app_src_push_buffer(source_, buffer) != GST_FLOW_OK) return;
        GstSample *sample = gst_app_sink_try_pull_sample(sink_, GST_MSECOND * 100);
        if (!sample) return;
        GstBuffer *decoded = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (decoded && gst_buffer_map(decoded, &map, GST_MAP_READ)) {
            GstCaps *caps = gst_sample_get_caps(sample);
            GstStructure *structure = gst_caps_get_structure(caps, 0);
            int width = 0, height = 0;
            gst_structure_get_int(structure, "width", &width);
            gst_structure_get_int(structure, "height", &height);
            if (width > 0 && height > 0 && map.size >= static_cast<size_t>(width * height * 3)) {
                cv::Mat image(height, width, CV_8UC3, map.data);
                std_msgs::msg::Header header = msg->header;
                publisher_->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
            }
            gst_buffer_unmap(decoded, &map);
        }
        gst_sample_unref(sample);
    }
    std::string topic_, image_topic_, pipeline_description_;
    GstElement *pipeline_ = nullptr;
    GstAppSrc *source_ = nullptr;
    GstAppSink *sink_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<GstreamerDecoderNode>()); }
    catch (const std::exception &exc) { fprintf(stderr, "%s\n", exc.what()); }
    rclcpp::shutdown();
    return 0;
}
