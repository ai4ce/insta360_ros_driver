#ifndef DUAL_FISHEYE2PERSPECTIVE_HPP
#define DUAL_FISHEYE2PERSPECTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unsupported/Eigen/CXX11/Tensor>

// 3D coordinate view: (height*width, 3) matrix where each row = [X, Y, Z]
typedef Eigen::TensorMap<Eigen::Tensor<float, 3, Eigen::RowMajor>> Tensor3D;
typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Matrix;
typedef Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> RotationMatrix;
typedef Eigen::Map<Eigen::Vector3f> XYZ_Vector;
typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,0,Eigen::Stride<Eigen::Dynamic, 3>> CoordinateView;
typedef Eigen::Stride<Eigen::Dynamic, 3> Stride3;
class DualFisheye2PerspectiveNode : public rclcpp::Node
{
public:
    explicit DualFisheye2PerspectiveNode();
    ~DualFisheye2PerspectiveNode();

private:
    // Callback functions
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    
    // Initialization functions
    void loadParameters();
    void updateCameraParameters();
    void initMapping(int img_height, int img_width);
    
    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr dual_fisheye_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr camera_orientation_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr perspective_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle;
    tf2::Quaternion camera_orientation_quaternion_;
    bool new_orientation_ = false;
    // Parameters
    float cx_offset_;
    float cy_offset_;
    float tx_, ty_, tz_;
    float roll_, pitch_, yaw_;
    bool gpu_enabled_;
    int out_width_;
    int out_height_;
    float horizontal_fov_;
    float vertical_fov_;
    int crop_size_;
    // Camera parameters
    float cx_, cy_;
    cv::Matx33f back_to_front_rotation_;
    cv::Vec3f back_to_front_translation_;
    
    // Mapping matrices
    cv::Mat full_map_x_;
    cv::Mat full_map_y_;
    cv::Mat opencv_coordinates;
    cv::Matx33f camera_orientation_matrix_ = cv::Matx33f::eye();
    // Images
    cv::Mat perspective_img;
    // init mapping matrices
    cv::Mat x_grid, y_grid, x_range, y_range;
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> back_mask_;
    // Vectorized fisheye projection arrays
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> r_array_, r_fisheye_array_;
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> u_array_, v_array_;
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> rot_u_array_, rot_v_array_;
    // State management
    std::atomic<bool> maps_initialized_;
    std::atomic<bool> params_changed_;
    int img_height_;
    int img_width_;
    
    // Thread safety
    std::mutex processing_mutex_;
};

#endif // DUAL_FISHEYE2PERSPECTIVE_HPP