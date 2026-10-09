#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <chrono>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <memory>
#include <string>

class WebcamEquirectangularNode : public rclcpp::Node
{
public:
    WebcamEquirectangularNode()
        : Node("webcam_equirectangular")
    {
        declare_parameter("device", "/dev/video0");
        declare_parameter("width", 1920);
        declare_parameter("height", 960);
        declare_parameter("fps", 30.0);
        declare_parameter("topic", "/equirectangular/image");
        declare_parameter("pixel_format", "MJPG");

        device_ = get_parameter("device").as_string();
        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        fps_ = get_parameter("fps").as_double();
        topic_ = get_parameter("topic").as_string();
        pixel_format_ = get_parameter("pixel_format").as_string();

        if (width_ <= 0 || height_ <= 0 || fps_ <= 0.0 ||
            std::abs(static_cast<double>(width_) / height_ - 2.0) > 0.01) {
            throw std::invalid_argument("width and height must define a 2:1 image and fps must be positive");
        }

        capture_.open(device_, cv::CAP_V4L2);
        if (!capture_.isOpened()) {
            throw std::runtime_error("Could not open webcam device: " + device_);
        }

        capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        capture_.set(cv::CAP_PROP_FPS, fps_);
        if (!pixel_format_.empty()) {
            capture_.set(cv::CAP_PROP_FOURCC,
                         cv::VideoWriter::fourcc(pixel_format_[0], pixel_format_[1],
                                                 pixel_format_[2], pixel_format_[3]));
        }

        const int actual_width = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
        const int actual_height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (actual_width <= 0 || actual_height <= 0 ||
            std::abs(static_cast<double>(actual_width) / actual_height - 2.0) > 0.01) {
            throw std::runtime_error("Webcam did not provide a 2:1 mode; requested " +
                                     std::to_string(width_) + "x" + std::to_string(height_) +
                                     ", received " + std::to_string(actual_width) + "x" +
                                     std::to_string(actual_height));
        }

        publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
            std::bind(&WebcamEquirectangularNode::capture_frame, this));

        RCLCPP_INFO(get_logger(), "Publishing %dx%d webcam frames from %s to %s",
                    actual_width, actual_height, device_.c_str(), topic_.c_str());
    }

private:
    void capture_frame()
    {
        cv::Mat frame;
        if (!capture_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Failed to read a frame from %s", device_.c_str());
            return;
        }

        if (frame.cols <= 0 || frame.rows <= 0 ||
            std::abs(static_cast<double>(frame.cols) / frame.rows - 2.0) > 0.01) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Ignoring non-2:1 webcam frame (%dx%d)", frame.cols, frame.rows);
            return;
        }

        cv::Mat rgb;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
        } else {
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        }

        cv_bridge::CvImage message;
        message.header.stamp = now();
        message.header.frame_id = "camera_frame";
        message.encoding = "rgb8";
        message.image = rgb;
        publisher_->publish(*message.toImageMsg());
    }

    std::string device_;
    std::string topic_;
    std::string pixel_format_;
    int width_;
    int height_;
    double fps_;
    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<WebcamEquirectangularNode>());
    } catch (const std::exception &error) {
        fprintf(stderr, "webcam_equirectangular failed: %s\n", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
