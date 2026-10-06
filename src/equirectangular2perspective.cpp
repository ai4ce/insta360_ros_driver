#include "equirectangular2perspective.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <numeric>
#include <thread>
#include <chrono>

Equirectangular2PerspectiveNode::Equirectangular2PerspectiveNode()
    : Node("equirectangular2perspective_node"),
      maps_initialized_(false),
      params_changed_(true),
        img_height_(0),
        img_width_(0)
{
    // Declare parameters
    declare_parameter("out_width", 1920);
    declare_parameter("out_height", 960);
    declare_parameter("horizontal_fov", 120.0);
    declare_parameter("vertical_fov", 81.7867893);
    
    // Load parameters
    loadParameters();
    
    RCLCPP_INFO(get_logger(), "C++ equirectangular-to-perspective node");
    
    
    // Add parameter callback
    params_callback_handle = add_on_set_parameters_callback(
        std::bind(&Equirectangular2PerspectiveNode::parametersCallback, this, std::placeholders::_1));
    
    // Configure QoS
    auto qos = rclcpp::QoS(1).reliable();
    
    // Create publishers and subscribers
    equirectangular_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/equirectangular/image", qos,
        std::bind(&Equirectangular2PerspectiveNode::imageCallback, this, std::placeholders::_1));

    camera_orientation_sub_ = create_subscription<geometry_msgs::msg::Quaternion>(
        "/camera_orientation/quaternion", qos,
        [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) {
            tf2::fromMsg(*msg, camera_orientation_quaternion_);

            tf2::Matrix3x3 matrix(camera_orientation_quaternion_);
            
            camera_orientation_matrix_ = cv::Matx33d(
            matrix[0][0], matrix[0][1], matrix[0][2],
            matrix[1][0], matrix[1][1], matrix[1][2],
            matrix[2][0], matrix[2][1], matrix[2][2]);
            new_orientation_ = true;
        });
    
    perspective_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/perspective/image", qos);
}

Equirectangular2PerspectiveNode::~Equirectangular2PerspectiveNode()
{
}

void Equirectangular2PerspectiveNode::loadParameters()
{
    try {
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();
        horizontal_fov_ = get_parameter("horizontal_fov").as_double();
        vertical_fov_ = get_parameter("vertical_fov").as_double();

        RCLCPP_INFO(get_logger(), "Loaded parameters from ROS parameter server");
        RCLCPP_INFO(get_logger(), "  Output size: %dx%d", out_width_, out_height_);
        RCLCPP_INFO(get_logger(), "  Horizontal FOV: %.1f", horizontal_fov_);
        RCLCPP_INFO(get_logger(), "  Vertical FOV: %.1f", vertical_fov_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error loading parameters: %s", e.what());
        
        throw;
    }
}

void Equirectangular2PerspectiveNode::initMapping(int img_height, int img_width)
{
    RCLCPP_INFO(get_logger(), "Initializing perspective projection from %dx%d equirectangular image to %dx%d",
                img_width, img_height, out_width_, out_height_);
    img_height_ = img_height;
    img_width_ = img_width;
    

    
    // Create output coordinate grids
    
    x_range = cv::Mat(1, out_width_, CV_32F);
    y_range = cv::Mat(out_height_, 1, CV_32F);
    
    // Fill with sequential values using direct pointer access (faster than .at<>)
    std::iota(x_range.ptr<float>(0), x_range.ptr<float>(0) + out_width_, 0.0);
    std::iota(y_range.ptr<float>(0), y_range.ptr<float>(0) + out_height_, 0.0);
    
    cv::repeat(x_range, out_height_, 1, x_grid);
    cv::repeat(y_range, 1, out_width_, y_grid);
    
    // Convert to spherical coordinates - vectorized using Eigen tensor with strided channel views
    // Note: x=0 corresponds to lon=-π, x=out_width-1 corresponds to lon=π*(out_width-1)/out_width
    float tan_horizontal = tan(horizontal_fov_ / 2.0 * M_PI / 180.0);
    float tan_vertical = tan(vertical_fov_ / 2.0 * M_PI / 180.0);
    
    opencv_coordinates = cv::Mat(out_height_, out_width_, CV_32FC3);
    // Eigen tensor (height, width, 3 channels) in row-major
    Tensor3D eigen_coordinates(opencv_coordinates.ptr<float>(),out_height_, out_width_, 3);
    
    // Create strided Matrix views for each channel (X, Y, Z) - interleaved layout
    CoordinateView eigen_X_coordinate(eigen_coordinates.data() + 0, out_height_, out_width_, Stride3(out_width_ * 3, 3));
    CoordinateView eigen_Y_coordinate(eigen_coordinates.data() + 1, out_height_, out_width_, Stride3(out_width_ * 3, 3));
    CoordinateView eigen_Z_coordinate(eigen_coordinates.data() + 2, out_height_, out_width_, Stride3(out_width_ * 3, 3));
    
    // Create grid views
    Matrix x_grid_view(x_grid.ptr<float>(), out_height_, out_width_);
    Matrix y_grid_view(y_grid.ptr<float>(), out_height_, out_width_);
    
    // Vectorized coordinate generation - directly to strided views
    eigen_X_coordinate = (x_grid_view / (float)out_width_) * 2.0 * tan_horizontal - tan_horizontal;
    eigen_Y_coordinate = (y_grid_view / (float)out_height_) * 2.0 * tan_vertical - tan_vertical;
    eigen_Z_coordinate.setConstant(1.0);
    
    
    cv::transform(opencv_coordinates, opencv_coordinates, camera_orientation_matrix_);
    
    
    full_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
    full_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);

    
    latitude = eigen_Y_coordinate.unaryExpr([](float value) {
        return std::asin(value);
    });
    longitude = eigen_X_coordinate.binaryExpr(
        eigen_Z_coordinate,
        [](float x, float z) {
            return std::atan2(x, z);
        });

    Matrix (full_map_x_.ptr<float>(), out_height_, out_width_) = (longitude + M_PI) / (2.0 * M_PI) * img_width;
    Matrix (full_map_y_.ptr<float>(), out_height_, out_width_) = (M_PI / 2.0 - latitude) / M_PI * img_height;
    
    maps_initialized_ = true;
    new_orientation_ = false;
    
    RCLCPP_INFO(get_logger(), "Mapping matrices initialization complete");
}


void Equirectangular2PerspectiveNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr equirectangular_msg)
{
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(equirectangular_msg, "rgb8");
        int rows_size=cv_ptr->image.rows;
        int cols_size=cv_ptr->image.cols;
        if (!maps_initialized_ || params_changed_ || new_orientation_ ||
                rows_size != img_height_ || cols_size != img_width_) {
                initMapping(rows_size, cols_size);
                params_changed_ = false;
            }
        auto start_time = now();
        // 3. Single Remap (Directly from Raw to Perspective)
        cv::remap(cv_ptr->image, perspective_img, full_map_x_, full_map_y_, cv::INTER_LINEAR);

        

        
        // Publish result
        if (perspective_pub_) {
            cv_bridge::CvImage out_msg;
            out_msg.header = equirectangular_msg->header;
            out_msg.encoding = "rgb8";
            out_msg.image = perspective_img;
            perspective_pub_->publish(*out_msg.toImageMsg());
        }
        
        auto process_time = (now() - start_time).seconds();
        RCLCPP_DEBUG(get_logger(), "Processing time: %.3f seconds", process_time);
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error processing images: %s", e.what());
    }
}

rcl_interfaces::msg::SetParametersResult Equirectangular2PerspectiveNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &param : parameters) {
        if (param.get_name() == "out_width") {
            out_width_ = param.as_int();
        } else if (param.get_name() == "out_height") {
            out_height_ = param.as_int();
        } else if (param.get_name() == "horizontal_fov") {
            horizontal_fov_ = param.as_double();
        } else if (param.get_name() == "vertical_fov") {
            vertical_fov_ = param.as_double();
        }
    }
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    params_changed_ = true;
    return result;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Equirectangular2PerspectiveNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
