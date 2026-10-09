#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "gstreamer_ros_sink.hpp"

class GstreamerEncoderNode : public rclcpp::Node {
public:
    GstreamerEncoderNode() : Node("gstreamer_encoder") {
        declare_parameter("input_topic", "/equirectangular/image");
        declare_parameter("output_topic", "/equirectangular/image/h264");
        declare_parameter("transport", "ros");
        declare_parameter("pipeline", std::string("appsrc ! videoconvert ! x264enc tune=zerolatency ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5000"));
        declare_parameter("ros_pipeline", std::string("appsrc name=ros_source is-live=true ! videoconvert ! x264enc tune=zerolatency ! h264parse ! appsink name=ros_sink sync=false"));
        declare_parameter("fps", 30.0);
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        transport_ = get_parameter("transport").as_string();
        pipeline_description_ = get_parameter("pipeline").as_string();
        ros_pipeline_ = get_parameter("ros_pipeline").as_string();
        fps_ = get_parameter("fps").as_double();
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&GstreamerEncoderNode::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Encoding %s using %s transport", input_topic_.c_str(), transport_.c_str());
    }
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr message) {
        try {
            auto image = cv_bridge::toCvShare(message, "bgr8")->image;
            if (transport_ == "ros") {
                if (!ros_sink_) {
                    ros_sink_ = std::make_unique<GstreamerRosSink>(shared_from_this(), output_topic_);
                }
                ros_sink_->write(image, message->header, ros_pipeline_, fps_);
                return;
            }
            if (!writer_.isOpened() && !writer_.open(pipeline_description_, cv::CAP_GSTREAMER, 0, fps_, image.size(), true)) {
                RCLCPP_ERROR(get_logger(), "Unable to open direct GStreamer pipeline");
                return;
            }
            writer_.write(image);
        } catch (const std::exception& exc) {
            RCLCPP_ERROR(get_logger(), "Encoding failed: %s", exc.what());
        }
    }
    std::string input_topic_, output_topic_, transport_, pipeline_description_, ros_pipeline_;
    double fps_;
    cv::VideoWriter writer_;
    std::unique_ptr<GstreamerRosSink> ros_sink_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GstreamerEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
