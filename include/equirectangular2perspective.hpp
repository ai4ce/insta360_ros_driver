#ifndef EQUIRECTANGULAR2PERSPECTIVE_HPP
#define EQUIRECTANGULAR2PERSPECTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unsupported/Eigen/CXX11/Tensor>

typedef Eigen::TensorMap<Eigen::Tensor<float, 3, Eigen::RowMajor>> Tensor3D;
typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Matrix;
typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride<Eigen::Dynamic, 3>> CoordinateView;
typedef Eigen::Stride<Eigen::Dynamic, 3> Stride3;

class Equirectangular2PerspectiveNode : public rclcpp::Node
{
public:
    explicit Equirectangular2PerspectiveNode();
    ~Equirectangular2PerspectiveNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    void loadParameters();
    void initMapping(int img_height, int img_width);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr equirectangular_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr camera_orientation_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr perspective_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle;
    tf2::Quaternion camera_orientation_quaternion_;
    bool new_orientation_ = false;

    int out_width_;
    int out_height_;
    double horizontal_fov_;
    double vertical_fov_;

    cv::Mat full_map_x_;
    cv::Mat full_map_y_;
    cv::Mat opencv_coordinates;
    cv::Matx33f camera_orientation_matrix_ = cv::Matx33f::eye();
    cv::Mat perspective_img;
    cv::Mat x_grid, y_grid, x_range, y_range;
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> longitude, latitude;

    std::atomic<bool> maps_initialized_;
    std::atomic<bool> params_changed_;
    int img_height_;
    int img_width_;
};

#endif // EQUIRECTANGULAR2PERSPECTIVE_HPP