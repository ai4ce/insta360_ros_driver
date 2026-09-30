#ifndef GSTREAMER_ROS_SINK_HPP
#define GSTREAMER_ROS_SINK_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/core.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <memory>
#include <string>

class GstreamerRosSink
{
public:
    GstreamerRosSink(const rclcpp::Node::SharedPtr& node, const std::string& topic);
    ~GstreamerRosSink();

    bool write(const cv::Mat& image, const std_msgs::msg::Header& header,
               const std::string& pipeline_description, double fps);
    void close();

private:
    bool open(const cv::Size& image_size, const std::string& pipeline_description,
              double fps);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    GstElement* pipeline_ = nullptr;
    GstAppSrc* appsrc_ = nullptr;
    GstAppSink* appsink_ = nullptr;
    cv::Size image_size_;
    double fps_ = 0.0;
    guint64 frame_index_ = 0;
};

#endif
