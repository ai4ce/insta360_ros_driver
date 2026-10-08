#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "insta360_ros_driver/msg/camera_preview_info.hpp"

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        // Publisher for the compressed H.264 video stream
        compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/image/compressed", 
            rclcpp::QoS(10)
        );

        // Publisher for IMU data (remains the same)
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // We only care about the main video stream (index 0)
        if (stream_index == 0 && size > 0 && compressed_pub_) {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

            // Set the header
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "camera_frame";

            // Set the format to H.264
            // The subscriber will need to know this to select the correct decoder.
            msg->format = "h264";

            // Copy the compressed video data directly into the message
            msg->data.assign(data, data + size);

            compressed_pub_->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            
            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<insta360_ros_driver::msg::CameraPreviewInfo>::SharedPtr metadata_pub_;

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    ~CameraWrapper() {
        if (cam) {
            cam->Close();
        }
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;

        rclcpp::QoS metadata_qos(1);
        metadata_qos.reliable().transient_local();
        metadata_pub_ = node_->create_publisher<insta360_ros_driver::msg::CameraPreviewInfo>(
            "/insta360/camera_preview_info", metadata_qos);

        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");

        auto preview = cam->GetPreviewParam();

        auto metadata = std::make_unique<insta360_ros_driver::msg::CameraPreviewInfo>();
        metadata->header.stamp = node_->get_clock()->now();
        metadata->header.frame_id = "camera_frame";
        metadata->camera_name = preview.camera_name;
        metadata->decode_type = preview.encode_type == ins_camera::VideoEncodeType::H265 ? "h265" : "h264";
        metadata->crop_src_width = preview.crop_info.src_width;
        metadata->crop_src_height = preview.crop_info.src_height;
        metadata->crop_dst_width = preview.crop_info.dst_width;
        metadata->crop_dst_height = preview.crop_info.dst_height;
        metadata->crop_offset_x = preview.crop_info.crop_offset_x;
        metadata->crop_offset_y = preview.crop_info.crop_offset_y;
        metadata->camera_offset = preview.offset;
        metadata_pub_->publish(std::move(metadata));
        RCLCPP_INFO(node_->get_logger(), "Published camera preview metadata for %s", preview.camera_name.c_str());
        discovery.FreeDeviceDescriptors(list);

        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>(node_);
        cam->SetStreamDelegate(delegate);

        auto start = time(NULL);

        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0; //no offset from UTC

        cam->SyncLocalTimeToCamera(utc_time,offset_time);       
        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_1920_960P30; //Change this line to edit the resolution
        //Possible resolutions (results may vary per model) are:
        //RES_3840_1920P30
        //RES_2560_1280P30
        //RES_1152_1152P30 (this will give 2304 x 1152 at 30 FPS)
        //RES_1920_960P30  
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_publisher");
    
    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}