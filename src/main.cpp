#include <iostream>
#include <thread>
#include <regex>
#include <opencv2/opencv.hpp>

#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include "camera/ins_types.h"

#include "stream/stream_delegate.h"
#include "stream/stream_types.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <atomic>
#include <signal.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

std::atomic<bool> shutdown_requested(false);
void signal_handler(int signal) {
    if (signal == SIGINT) {
        shutdown_requested = true;
    }
}
class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    FILE* file1_;
    FILE* file2_;
    int64_t last_timestamp = 0;
    AVCodec* codec;
    AVCodecContext* codecCtx;
    AVFrame* avFrame;
    AVPacket* pkt;
    struct SwsContext* img_convert_ctx;

    ros::NodeHandle nh;
    ros::Publisher image_pub;

public:
    TestStreamDelegate(){
        image_pub = nh.advertise<sensor_msgs::Image>("insta_image_yuv",60);

        file1_ = fopen("./01.h264", "wb");
        file2_ = fopen("./02.h264", "wb");

        // avcodec_register_all();

        // Find the decoder for the h264
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            std::cerr << "Codec not found\n";
            exit(1);
        }

        codecCtx = avcodec_alloc_context3(codec);
        codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
        if (!codecCtx) {
            std::cerr << "Could not allocate video codec context\n";
            exit(1);
        }

        // Open codec
        if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
            std::cerr << "Could not open codec\n";
            exit(1);
        }

        avFrame = av_frame_alloc();
        pkt = av_packet_alloc();
    }

    ~TestStreamDelegate() {
        fclose(file1_);
        fclose(file2_);
        av_frame_free(&avFrame);
        av_packet_free(&pkt);
        avcodec_free_context(&codecCtx);
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        //std::cout << "on video frame:" << size << ";" << timestamp << std::endl;
        if (stream_index == 0) {
            // Feed data into packet
            pkt->data = const_cast<uint8_t*>(data);
            pkt->size = size;

            // Send the packet to the decoder
            if (avcodec_send_packet(codecCtx, pkt) == 0) {
                // Receive frame from decoder
                while (avcodec_receive_frame(codecCtx, avFrame) == 0) {
                    int width = avFrame->width;
                    int height = avFrame->height;
                    int chromaHeight = height / 2;
                    int chromaWidth = width / 2;

                    // Create a single Mat to hold all three planes
                    cv::Mat yuv(height + chromaHeight, width, CV_8UC1);

                    // Copy the Y plane
                    memcpy(yuv.data, avFrame->data[0], width * height);

                    // Copy the U plane
                    memcpy(yuv.data + width * height, avFrame->data[1], chromaWidth * chromaHeight);

                    // Copy the V plane
                    memcpy(yuv.data + width * height + chromaWidth * chromaHeight, avFrame->data[2], chromaWidth * chromaHeight);
                    
                    sensor_msgs::Image msg;
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = "camera_frame";
                    msg.height = yuv.rows;
                    msg.width = yuv.cols;
                    msg.encoding = "8UC1";
                    msg.is_bigendian = false;
                    msg.step = yuv.cols * yuv.elemSize();
                    msg.data.assign(yuv.datastart, yuv.dataend);

                    this->image_pub.publish(msg);
                }
            }
        }
    }
    cv::Mat avframeToCvmat(const AVFrame *frame) {
        int width = frame->width;
        int height = frame->height;

        // Create an OpenCV Mat with the same dimensions as the AVFrame
        cv::Mat yuv(height + height / 2, width, CV_8UC1, frame->data[0]);

        // Convert the YUV420P frame to BGR
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV420p2BGR);

        return bgr;
    }
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        //for (auto& gyro : data) {
        //	if (gyro.timestamp - last_timestamp > 2) {
        //		fprintf(file1_, "timestamp:%lld package_size = %d  offtimestamp = %lld gyro:[%f %f %f] accel:[%f %f %f]\n", gyro.timestamp, data.size(), gyro.timestamp - last_timestamp, gyro.gx, gyro.gy, gyro.gz, gyro.ax, gyro.ay, gyro.az);
        //	}
        //	last_timestamp = gyro.timestamp;
        //}
    }
    void OnExposureData(const ins_camera::ExposureData& data) override {
        //fprintf(file2_, "timestamp:%lld shutter_speed_s:%f\n", data.timestamp, data.exposure_time);
    }
};

class CameraWrapper {
    private:
        std::shared_ptr<ins_camera::Camera> cam;
    public:
        CameraWrapper(int argc, char* argv[]){
            ros::init(argc, argv, "insta");
            ROS_ERROR("Opened Camera\n");
        }
        ~CameraWrapper(){
            ROS_ERROR("Closing Camera\n");
            this->cam->Close();
        }

        int run_camera()
        {
            ins_camera::DeviceDiscovery discovery;
            auto list = discovery.GetAvailableDevices();
            for (int i = 0; i < list.size(); ++i) {
                auto desc = list[i];
                std::cout << "serial:" << desc.serial_number << "\t"
                    << "camera type:" << int(desc.camera_type) << "\t"
                    << "lens type:" << int(desc.lens_type) << std::endl;
            }

            if (list.size() <= 0) {
                std::cerr << "no device found." << std::endl;
                return -1;
            }

            this->cam = std::make_shared<ins_camera::Camera>(list[0].info);
            //ins_camera::Camera cam(list[0].info);
            if (!this->cam->Open()) {
                std::cerr << "failed to open camera" << std::endl;
                return -1;
            }

            std::cout << "http base url:" << this->cam->GetHttpBaseUrl() << std::endl;

            std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>();
            this->cam->SetStreamDelegate(delegate);

            discovery.FreeDeviceDescriptors(list);

            std::cout << "Successfully opened camera..." << std::endl;

            auto camera_type = this->cam->GetCameraType();

            auto start = time(NULL);
            this->cam->SyncLocalTimeToCamera(start);
            
            ins_camera::LiveStreamParam param;
            param.video_resolution = ins_camera::VideoResolution::RES_2176_1088P30;
            param.video_bitrate = 1024 * 1024* 10;
            param.enable_audio = false;
            param.using_lrv = false;

            do{

            } while (!this->cam->StartLiveStreaming(param));
            std::cout << "successfully started live stream" << std::endl;
        }
};

int main(int argc, char* argv[]) {
    // Register signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    {
        CameraWrapper camera(argc, argv);
        camera.run_camera();

        while (ros::ok() && !shutdown_requested.load()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Reduce CPU usage
        }
    }

    ros::shutdown();
    return 0;
}