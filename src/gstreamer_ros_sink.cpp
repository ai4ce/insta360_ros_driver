#include "gstreamer_ros_sink.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <stdexcept>
#include <mutex>

GstreamerRosSink::GstreamerRosSink(const rclcpp::Node::SharedPtr& node,
                                   const std::string& topic)
    : node_(node)
{
    static std::once_flag gstreamer_initialized;
    std::call_once(gstreamer_initialized, []() {
        gst_init(nullptr, nullptr);
    });
    publisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 10);
}

GstreamerRosSink::~GstreamerRosSink()
{
    close();
}

bool GstreamerRosSink::open(const cv::Size& image_size,
                            const std::string& pipeline_description,
                            double fps)
{
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (!pipeline_ || error) {
        RCLCPP_ERROR(node_->get_logger(), "Could not create GStreamer ROS pipeline: %s",
                     error ? error->message : "unknown error");
        if (error) {
            g_error_free(error);
        }
        close();
        return false;
    }

    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "ros_source"));
    appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "ros_sink"));
    if (!appsrc_ || !appsink_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "ROS GStreamer pipeline must contain appsrc name=ros_source and appsink name=ros_sink");
        close();
        return false;
    }

    GstCaps* caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, image_size.width,
        "height", G_TYPE_INT, image_size.height,
        "framerate", GST_TYPE_FRACTION, static_cast<int>(fps), 1,
        nullptr);
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
    gst_app_src_set_stream_type(appsrc_, GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_format(appsrc_, GST_FORMAT_TIME);
    gst_app_sink_set_drop(appsink_, true);
    gst_app_sink_set_max_buffers(appsink_, 1);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(node_->get_logger(), "Could not start GStreamer ROS pipeline");
        close();
        return false;
    }

    image_size_ = image_size;
    fps_ = fps;
    frame_index_ = 0;
    return true;
}

bool GstreamerRosSink::write(const cv::Mat& image,
                             const std_msgs::msg::Header& header,
                             const std::string& pipeline_description,
                             double fps)
{
    if (image.empty() || fps <= 0.0) {
        return false;
    }
    if (!pipeline_ && !open(image.size(), pipeline_description, fps)) {
        return false;
    }
    if (image.size() != image_size_ || fps != fps_) {
        close();
        if (!open(image.size(), pipeline_description, fps)) {
            return false;
        }
    }

    const size_t size = image.total() * image.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(buffer, 0, image.data, size);
    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(frame_index_, GST_SECOND, fps_);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, fps_);
    ++frame_index_;

    if (gst_app_src_push_buffer(appsrc_, buffer) != GST_FLOW_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Could not push frame into GStreamer ROS pipeline");
        return false;
    }

    GstSample* sample = gst_app_sink_try_pull_sample(appsink_, GST_SECOND / 2);
    if (!sample) {
        return false;
    }

    GstBuffer* encoded_buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    const bool mapped = encoded_buffer && gst_buffer_map(encoded_buffer, &map, GST_MAP_READ);
    if (mapped) {
        auto message = std::make_unique<sensor_msgs::msg::CompressedImage>();
        message->header = header;
        message->format = "h264";
        message->data.assign(map.data, map.data + map.size);
        publisher_->publish(std::move(message));
        gst_buffer_unmap(encoded_buffer, &map);
    }
    gst_sample_unref(sample);
    return mapped;
}

void GstreamerRosSink::close()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
    if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}
