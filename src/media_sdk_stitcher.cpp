#include <ins_realtime_stitcher.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include "insta360_ros_driver/msg/camera_preview_info.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

class MediaSdkStitcherNode : public rclcpp::Node
{
public:
    MediaSdkStitcherNode()
        : Node("media_sdk_stitcher")
    {
        declare_parameter("input_video_topic", "/dual_fisheye/image/compressed");
        declare_parameter("input_imu_topic", "/imu/data_raw");
        declare_parameter("output_topic", "/equirectangular/image");
        declare_parameter("output_width", 1920);
        declare_parameter("output_height", 960);
        declare_parameter("stitch_type", "dynamicstitch");
        declare_parameter("enable_flowstate", false);
        declare_parameter("enable_direction_lock", false);
        declare_parameter("enable_deflicker", false);
        declare_parameter("enable_defringe", false);

        ins::InitEnv();
        ins::SetLogLevel(ins::InsLogLevel::WARNING);
        stitcher_ = std::make_shared<ins::RealTimeStitcher>();

        output_pub_ = create_publisher<sensor_msgs::msg::Image>(
            get_parameter("output_topic").as_string(), rclcpp::SensorDataQoS());
        video_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            get_parameter("input_video_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&MediaSdkStitcherNode::video_callback, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("input_imu_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&MediaSdkStitcherNode::imu_callback, this, std::placeholders::_1));
        rclcpp::QoS metadata_qos(1);
        metadata_qos.reliable().transient_local();
        metadata_sub_ = create_subscription<insta360_ros_driver::msg::CameraPreviewInfo>(
            "/insta360/camera_preview_info", metadata_qos,
            std::bind(&MediaSdkStitcherNode::metadata_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Waiting for camera preview metadata");
    }

    ~MediaSdkStitcherNode() override
    {
        if (stitcher_) {
            stitcher_->CancelStitch();
        }
    }

private:
    static int64_t stamp_to_us(const builtin_interfaces::msg::Time& stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000LL + stamp.nanosec / 1000;
    }

    void metadata_callback(const insta360_ros_driver::msg::CameraPreviewInfo::SharedPtr msg)
    {
        if (configured_) {
            return;
        }
        configure_stitcher(*msg);
        stitcher_->StartStitch();
        configured_ = true;
        RCLCPP_INFO(get_logger(), "MediaSDK configured automatically for %s", msg->camera_name.c_str());
    }

    void configure_stitcher(const insta360_ros_driver::msg::CameraPreviewInfo& preview)
    {
        ins::CameraInfo camera_info;
        camera_info.cameraName = preview.camera_name;
        const auto decode_type = preview.decode_type;
        camera_info.decode_type = decode_type == "h265"
            ? ins::VideoDecodeType::kH265 : ins::VideoDecodeType::kH264;
        camera_info.offset = preview.camera_offset;
        camera_info.window_crop_info_.src_width = preview.crop_src_width;
        camera_info.window_crop_info_.src_height = preview.crop_src_height;
        camera_info.window_crop_info_.dst_width = preview.crop_dst_width;
        camera_info.window_crop_info_.dst_height = preview.crop_dst_height;
        camera_info.window_crop_info_.crop_offset_x = preview.crop_offset_x;
        camera_info.window_crop_info_.crop_offset_y = preview.crop_offset_y;
        stitcher_->SetCameraInfo(camera_info);

        const auto stitch_type = get_parameter("stitch_type").as_string();
        if (stitch_type == "template") {
            stitcher_->SetStitchType(ins::STITCH_TYPE::TEMPLATE);
        } else if (stitch_type == "optflow") {
            stitcher_->SetStitchType(ins::STITCH_TYPE::OPTFLOW);
        } else if (stitch_type == "aistitch") {
            stitcher_->SetStitchType(ins::STITCH_TYPE::AIFLOW);
        } else {
            stitcher_->SetStitchType(ins::STITCH_TYPE::DYNAMICSTITCH);
        }
        stitcher_->SetOutputSize(
            get_parameter("output_width").as_int(),
            get_parameter("output_height").as_int());
        stitcher_->EnableFlowState(get_parameter("enable_flowstate").as_bool());
        stitcher_->EnableDirectionLock(get_parameter("enable_direction_lock").as_bool());
        stitcher_->EnableDeflicker(get_parameter("enable_deflicker").as_bool());
        stitcher_->EnableDefringe(get_parameter("enable_defringe").as_bool());
        stitcher_->SetStitchRealTimeDataCallback(
            [this](uint8_t* data[4], int linesize[4], int width, int height,
                   int format, int64_t timestamp) {
                publish_frame(data, linesize, width, height, format, timestamp);
            });
        stitcher_->SetStitchStateCallback(
            [this](int error, const char* info) {
                RCLCPP_ERROR(get_logger(), "MediaSDK stitch error %d: %s", error,
                             info ? info : "unknown");
            });
    }

    void video_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (!configured_) {
            return;
        }
        if (msg->format != "h264" && msg->format != "h265") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Expected h264/h265 input, got '%s'", msg->format.c_str());
            return;
        }
        stitcher_->HandleVideoData(
            msg->data.data(), msg->data.size(), stamp_to_us(msg->header.stamp),
            msg->format == "h265" ? 1 : 0, 0);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!configured_) {
            return;
        }
        ins::GyroData gyro{};
        gyro.timestamp = stamp_to_us(msg->header.stamp);
        gyro.ax = msg->linear_acceleration.x / 9.80665;
        gyro.ay = msg->linear_acceleration.y / 9.80665;
        gyro.az = msg->linear_acceleration.z / 9.80665;
        gyro.gx = msg->angular_velocity.x;
        gyro.gy = msg->angular_velocity.y;
        gyro.gz = msg->angular_velocity.z;
        stitcher_->HandleGyroData(std::vector<ins::GyroData>{gyro});
    }

    void publish_frame(uint8_t* data[4], int linesize[4], int width, int height,
                       int format, int64_t timestamp)
    {
        if (!data[0] || width <= 0 || height <= 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(publish_mutex_);
        const auto encoding = format == 0 ? "rgba8" : "bgra8";
        cv::Mat image(height, width, CV_8UC4, data[0], linesize[0]);
        cv_bridge::CvImage output;
        output.header.stamp.sec = static_cast<int32_t>(timestamp / 1000000);
        output.header.stamp.nanosec = static_cast<uint32_t>((timestamp % 1000000) * 1000);
        output.header.frame_id = "camera_frame";
        output.encoding = encoding;
        output.image = image.clone();
        output_pub_->publish(*output.toImageMsg());
    }

    std::shared_ptr<ins::RealTimeStitcher> stitcher_;
    bool configured_ = false;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr video_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<insta360_ros_driver::msg::CameraPreviewInfo>::SharedPtr metadata_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub_;
    std::mutex publish_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MediaSdkStitcherNode>());
    } catch (const std::exception& error) {
        fprintf(stderr, "MediaSDK stitcher failed: %s\n", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
