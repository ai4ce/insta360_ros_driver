#include "perspective.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <numeric>
#include <thread>
#include <chrono>

PerspectiveNode::PerspectiveNode()
    : Node("perspective_node"),
      maps_initialized_(false),
      params_changed_(true),
            img_height_(0),
            img_width_(0)
{
    // Declare parameters
    declare_parameter("cx_offset", 0.0);
    declare_parameter("cy_offset", 0.0);
    declare_parameter("crop_size", 960);
    declare_parameter("translation", std::vector<float>{0.0, 0.0, -0.105});
    declare_parameter("rotation_deg", std::vector<float>{-0.5, 0.0, 1.1});
    declare_parameter("gpu", true);
    declare_parameter("out_width", 1920);
    declare_parameter("out_height", 960);
    declare_parameter("horizontal_fov", 120.0);
    declare_parameter("vertical_fov", 81.7867893);
    
    // Load parameters
    loadParameters();
    
    // Log GPU settings (note: C++ version currently only supports CPU)
    RCLCPP_INFO(get_logger(), "C++ perspective node");
    
    
    // Add parameter callback
    params_callback_handle = add_on_set_parameters_callback(
        std::bind(&PerspectiveNode::parametersCallback, this, std::placeholders::_1));
    
    updateCameraParameters();
    
    // Configure QoS
    auto qos = rclcpp::QoS(1).reliable();
    
    // Create publishers and subscribers
    dual_fisheye_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", qos,
        std::bind(&PerspectiveNode::imageCallback, this, std::placeholders::_1));

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

PerspectiveNode::~PerspectiveNode()
{
}

void PerspectiveNode::loadParameters()
{
    try {
        cx_offset_ = get_parameter("cx_offset").as_double();
        cy_offset_ = get_parameter("cy_offset").as_double();
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();
        gpu_enabled_ = get_parameter("gpu").as_bool();
        horizontal_fov_ = get_parameter("horizontal_fov").as_double();
        vertical_fov_ = get_parameter("vertical_fov").as_double();
        crop_size_ = get_parameter("crop_size").as_int();

        auto translation = get_parameter("translation").as_double_array();
        tx_ = translation[0];
        ty_ = translation[1];
        tz_ = translation[2];
        
        auto rotation_deg = get_parameter("rotation_deg").as_double_array();
        roll_ = rotation_deg[0] * M_PI / 180.0;
        pitch_ = rotation_deg[1] * M_PI / 180.0;
        yaw_ = rotation_deg[2] * M_PI / 180.0;
        
        RCLCPP_INFO(get_logger(), "Loaded parameters from ROS parameter server");
        RCLCPP_INFO(get_logger(), "  Crop size: %d", crop_size_);
        RCLCPP_INFO(get_logger(), "  Center offset: (%.1f, %.1f)", cx_offset_, cy_offset_);
        RCLCPP_INFO(get_logger(), "  Translation: [%.3f, %.3f, %.3f]", tx_, ty_, tz_);
        RCLCPP_INFO(get_logger(), "  Rotation (deg): [%.1f, %.1f, %.1f]", 
                    rotation_deg[0], rotation_deg[1], rotation_deg[2]);
        RCLCPP_INFO(get_logger(), "  Output size: %dx%d", out_width_, out_height_);
        RCLCPP_INFO(get_logger(), "  Horizontal FOV: %.1f", horizontal_fov_);
        RCLCPP_INFO(get_logger(), "  Vertical FOV: %.1f", vertical_fov_);
        RCLCPP_INFO(get_logger(), "  GPU enabled: %s", gpu_enabled_ ? "true" : "false");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error loading parameters: %s", e.what());
        
        gpu_enabled_ = true;
        throw;
    }
}

void PerspectiveNode::updateCameraParameters()
{
    // Build rotation matrix
    cv::Matx33d Rx(
        1.0, 0.0, 0.0,
        0.0, cos(roll_), -sin(roll_),
        0.0, sin(roll_), cos(roll_)
    );
    
    cv::Matx33d Ry(
        cos(pitch_), 0.0, sin(pitch_),
        0.0, 1.0, 0.0,
        -sin(pitch_), 0.0, cos(pitch_)
    );
    
    cv::Matx33d Rz(
        cos(yaw_), -sin(yaw_), 0.0,
        sin(yaw_), cos(yaw_), 0.0,
        0.0, 0.0, 1.0
    );
    
    back_to_front_rotation_ = Rz * Ry * Rx;
    RotationMatrix rot_eigen(back_to_front_rotation_.val);
    rot_eigen.row(0) *= -1.0;
    back_to_front_translation_ = cv::Vec3d(tx_, ty_, tz_);
    
    if (maps_initialized_) {
        maps_initialized_ = false;
        RCLCPP_INFO(get_logger(), "Parameters updated, remapping will occur on next image");
    }
}

void PerspectiveNode::initMapping(int img_height, int img_width)
{
    RCLCPP_INFO(get_logger(), "Initializing perspective projection: fusing two %dx%d fisheye images to %dx%d",
                img_width, img_height, out_width_, out_height_);
    
    img_height_ = img_height;
    img_width_ = img_width;
    
    int current_crop_size = crop_size_;
    int y_offset_crop = 0;
    int x_offset_crop = 0;
    if (img_height_ != current_crop_size || img_width_ != current_crop_size) {
            int y_start = (img_height_ - current_crop_size) / 2;
            int x_start = (img_width_ - current_crop_size) / 2;
            
            if (y_start >= 0 && x_start >= 0 &&
                y_start + current_crop_size <= img_height_ &&
                x_start + current_crop_size <= img_width_) {
                img_height=current_crop_size;
                img_width=current_crop_size;
                y_offset_crop = (img_height_ - crop_size_) / 2;
                x_offset_crop = (img_width_ - crop_size_) / 2;
            } 
        } 

    cx_ = img_width / 2.0 + cx_offset_;
    cy_ = img_height / 2.0 + cy_offset_;
    
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
    

    
    // Create back_mask_ from Z channel (2D, matches image layout)
    back_mask_ = eigen_Z_coordinate < 0.0;
    
    full_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
    full_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);

    
    // Create Eigen maps to rotation and translation (OpenCV -> Eigen, zero-copy)
    RotationMatrix rot_eigen(back_to_front_rotation_.val);
    XYZ_Vector trans_eigen(back_to_front_translation_.val);
    

    
    // Apply transformation: loop over pixels, if front keep original, if back apply transformation
    for (int y = 0; y < out_height_; ++y) {
        for (int x = 0; x < out_width_; ++x) {
            if (back_mask_(y, x)) {
                // Back camera: apply rotation and translation
                XYZ_Vector coord(&eigen_coordinates(y, x, 0));
                coord = rot_eigen * coord + trans_eigen;
            }
            // Front camera: keep original coordinates (no action needed)
        }
    }
    

    
    // Vectorized fisheye projection calculations
    r_array_ = (eigen_X_coordinate.square() + eigen_Y_coordinate.square()).sqrt().max(1e-6);  // Clamp minimum value
    r_fisheye_array_ = 2.0 * r_array_.binaryExpr(
        eigen_Z_coordinate.abs(),
        [](float r, float z) { return std::atan2(r, z); }) / M_PI * (img_width / 2.0);
    u_array_ = cx_ + eigen_X_coordinate / r_array_ * r_fisheye_array_;
    v_array_ = cy_ + eigen_Y_coordinate / r_array_ * r_fisheye_array_;
    
    // Vectorized rotation calculations
    rot_u_array_ = back_mask_.select(v_array_,(img_height - 1.0) - v_array_);
    rot_v_array_ = back_mask_.select(u_array_,(img_width - 1.0) - u_array_);
    
    // Vectorized final coordinate calculations
    rot_u_array_ = rot_u_array_ + x_offset_crop;
    rot_v_array_ = rot_v_array_ + y_offset_crop;
    Matrix (full_map_x_.ptr<float>(), out_height_, out_width_) = back_mask_.select(rot_u_array_,img_width + rot_u_array_);
    Matrix (full_map_y_.ptr<float>(), out_height_, out_width_) = rot_v_array_;

    
    maps_initialized_ = true;
    new_orientation_ = false;
    
    RCLCPP_INFO(get_logger(), "Mapping matrices initialization complete");
}


void PerspectiveNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr dual_fisheye_msg)
{
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(dual_fisheye_msg, "rgb8");
        int rows_size=cv_ptr->image.rows;
        int cols_size=cv_ptr->image.cols/2;
        if (!maps_initialized_ || params_changed_ || new_orientation_ ||
                rows_size != img_height_ || cols_size != img_width_) {
                // Pass the full raw image dimensions
                initMapping(rows_size, cols_size);
                params_changed_ = false;
            }
        auto start_time = now();
        // 3. Single Remap (Directly from Raw to Perspective)
        cv::remap(cv_ptr->image, perspective_img, full_map_x_, full_map_y_, cv::INTER_LINEAR);
        

        
        // Publish result
        cv_bridge::CvImage out_msg;
        out_msg.header = dual_fisheye_msg->header;
        out_msg.encoding = "rgb8";
        out_msg.image = perspective_img;
        perspective_pub_->publish(*out_msg.toImageMsg());
        
        auto process_time = (now() - start_time).seconds();
        RCLCPP_DEBUG(get_logger(), "Processing time: %.3f seconds", process_time);
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error processing images: %s", e.what());
    }
}

rcl_interfaces::msg::SetParametersResult PerspectiveNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    bool update_needed = false;
    
for (const auto &param : parameters)
    {

        // Update member variables directly from the 'param' argument
        if (param.get_name() == "cx_offset") {
            cx_offset_ = param.as_double();
            update_needed = true;
        }
        else if (param.get_name() == "cy_offset") {
            cy_offset_ = param.as_double();
            update_needed = true;
        }
        else if (param.get_name() == "crop_size") {
            crop_size_ = param.as_int();
            update_needed = true;
        }
        else if (param.get_name() == "out_width") {
            out_width_ = param.as_int();
            update_needed = true;
        }
        else if (param.get_name() == "out_height") {
            out_height_ = param.as_int();
            update_needed = true;
        }
        else if (param.get_name() == "gpu") {
            gpu_enabled_ = param.as_bool();
            update_needed = true;
        }
        else if (param.get_name() == "translation") {
            auto translation = param.as_double_array();
            if (translation.size() >= 3) {
                tx_ = translation[0];
                ty_ = translation[1];
                tz_ = translation[2];
                update_needed = true;
            }
        }
        else if (param.get_name() == "rotation_deg") {
            auto rotation_deg = param.as_double_array();
            if (rotation_deg.size() >= 3) {
                roll_ = rotation_deg[0] * M_PI / 180.0;
                pitch_ = rotation_deg[1] * M_PI / 180.0;
                yaw_ = rotation_deg[2] * M_PI / 180.0;
                update_needed = true;
            }
        } else if (param.get_name() == "horizontal_fov") {
            horizontal_fov_ = param.as_double();
            update_needed = true;
        } else if (param.get_name() == "vertical_fov") {
            vertical_fov_ = param.as_double();
            update_needed = true;
        }
    }
    
    if (update_needed) {
        updateCameraParameters();
    }
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    params_changed_ = true;
    return result;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PerspectiveNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}