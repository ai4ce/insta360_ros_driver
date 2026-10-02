#include "equirectangular.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <thread>
#include <chrono>

EquirectangularNode::EquirectangularNode()
    : Node("equirectangular_node"),
      maps_initialized_(false),
      params_changed_(true),
      img_height_(0),
      img_width_(0)
{
    // Declare parameters
    declare_parameter("cx_offset", 0.0);
    declare_parameter("cy_offset", 0.0);
    declare_parameter("crop_size", 0);
    declare_parameter("translation", std::vector<double>{0.0, 0.0, -0.105});
    declare_parameter("rotation_deg", std::vector<double>{-0.5, 0.0, 1.1});
    declare_parameter("gpu", true);
    declare_parameter("out_width", 1920);
    declare_parameter("out_height", 960);
    
    // Load parameters
    loadParameters();
    
    // Log GPU settings (note: C++ version currently only supports CPU)
    RCLCPP_INFO(get_logger(), "C++ equirectangular node");
    
    
    // Add parameter callback
    params_callback_handle = add_on_set_parameters_callback(
        std::bind(&EquirectangularNode::parametersCallback, this, std::placeholders::_1));
    
    updateCameraParameters();
    
    // Configure QoS
    auto qos = rclcpp::QoS(1).reliable();
    
    // Create publishers and subscribers
    dual_fisheye_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", qos,
        std::bind(&EquirectangularNode::imageCallback, this, std::placeholders::_1));
    
    equirect_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/equirectangular/image", qos);
}

EquirectangularNode::~EquirectangularNode()
{
}

void EquirectangularNode::loadParameters()
{
    try {
        cx_offset_ = get_parameter("cx_offset").as_double();
        cy_offset_ = get_parameter("cy_offset").as_double();
        crop_size_ = get_parameter("crop_size").as_int();
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();
        gpu_enabled_ = get_parameter("gpu").as_bool();
        
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
        RCLCPP_INFO(get_logger(), "  GPU enabled: %s", gpu_enabled_ ? "true" : "false");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error loading parameters: %s", e.what());
        gpu_enabled_ = true;
        throw;
    }
}

void EquirectangularNode::updateCameraParameters()
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
    back_to_front_translation_ = cv::Vec3d(tx_, ty_, tz_);
    
    if (maps_initialized_) {
        maps_initialized_ = false;
        RCLCPP_INFO(get_logger(), "Parameters updated, remapping will occur on next image");
    }
}

void EquirectangularNode::initMapping(int img_height, int img_width)
{
    RCLCPP_INFO(get_logger(), "Initializing equirectangular projection: fusing two %dx%d fisheye images to %dx%d",
                img_width, img_height, out_width_, out_height_);
    
    img_height_ = img_height;
    img_width_ = img_width;
    int current_crop_size = img_height - crop_size_;
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
                y_offset_crop = (img_height_ - current_crop_size) / 2;
                x_offset_crop = (img_width_ - current_crop_size) / 2;
            } 
        } 

    cx_ = img_width / 2.0 + cx_offset_;
    cy_ = img_height / 2.0 + cy_offset_;
    
    // Create output coordinate grids
    
    x_range = cv::Mat::zeros(1, out_width_, CV_32F);
    y_range = cv::Mat::zeros(out_height_, 1, CV_32F);
    
    for (int i = 0; i < out_width_; ++i) {
        x_range.at<float>(0, i) = static_cast<float>(i);
    }
    for (int i = 0; i < out_height_; ++i) {
        y_range.at<float>(i, 0) = static_cast<float>(i);
    }
    
    cv::repeat(x_range, out_height_, 1, x_grid);
    cv::repeat(y_range, 1, out_width_, y_grid);
    
    // Convert to spherical coordinates
    // Note: x=0 corresponds to lon=-π, x=out_width-1 corresponds to lon=π*(out_width-1)/out_width
    longitude = (x_grid / (float)out_width_) * 2 * M_PI - M_PI;
    latitude = (y_grid / (float)out_height_) * M_PI - M_PI / 2;
    
    
    
    cv::exp(-latitude, cos_lat); // Using exp(-x) as intermediate for cos calculation
    cos_lat = (1 - cos_lat) / (1 + cos_lat); // Convert to cos
    cv::sqrt(1 - cos_lat.mul(cos_lat), sin_lat);
    
    cv::exp(-longitude, cos_lon);
    cos_lon = (1 - cos_lon) / (1 + cos_lon);
    cv::sqrt(1 - cos_lon.mul(cos_lon), sin_lon);
    
    // Correct calculation
    
    for (int y = 0; y < out_height_; ++y) {
        for (int x = 0; x < out_width_; ++x) {
            float lat = latitude.at<float>(y, x);
            float lon = longitude.at<float>(y, x);
            cos_lat.at<float>(y, x) = cos(lat);
            sin_lat.at<float>(y, x) = sin(lat);
            cos_lon.at<float>(y, x) = cos(lon);
            sin_lon.at<float>(y, x) = sin(lon);
        }
    }
    
    X = cos_lat.mul(sin_lon);
    Y = sin_lat;
    Z = cos_lat.mul(cos_lon);
    
    // Create mask
    front_mask_ = Z >= 0;

    
    full_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
    full_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);


    for (int y = 0; y < out_height_; ++y) {
        for (int x = 0; x < out_width_; ++x) {
            
            float X_val, Y_val, Z_val; // The calculated coordinates in the fisheye lens

            // ... (Perform your existing 3D -> 2D projection math here) ...
            // Result is (u, v) relative to the lens center (cx_, cy_)
            // Note: Ensure cx_, cy_ are relative to the *crop*, not the full image yet.
            if (front_mask_.at<uchar>(y, x)) {
                X_val = X.at<float>(y, x);
                Y_val = Y.at<float>(y, x);
                Z_val = Z.at<float>(y, x);

            } else {
                cv::Vec3d point(X.at<float>(y, x), Y.at<float>(y, x), Z.at<float>(y, x));
                
                // Transform point
                cv::Matx point_mat = cv::Matx(point);
                cv::Matx transformed = back_to_front_rotation_ * point_mat + back_to_front_translation_;
                
                X_val = -transformed(0,0);
                Y_val = transformed(1,0);
                Z_val = transformed(2,0);
                
                
            }
            float r = sqrt(X_val * X_val + Y_val * Y_val);
            if (r < 1e-6) r = 1e-6;
                
            float theta = atan2(r, fabs(Z_val));
            float r_fisheye = 2 * theta / M_PI * (img_width / 2.0);
                
            float u = cx_ + X_val / r * r_fisheye;
            float v = cy_ + Y_val / r * r_fisheye;

            // --- OPTIMIZATION: BAKE ROTATION AND TRANSLATION ---
            
            // 1. Handle Rotation (simulating cv::rotate)
            // If the front image was rotated 90 deg Counter-Clockwise:
            // New x' = y
            // New y' = -x (plus offset)
            float rot_u, rot_v;
            // Calculate offsets to center the crop in the raw image half
            

            
            if (front_mask_.at<uchar>(y, x)) {
                // Apply Front Rotation (90 CCW) math to coordinates
                rot_u = (img_height - 1) - v;
                rot_v = u;
            } else {
                // Apply Back Rotation (90 CW) math
                
                rot_u = v; 
                rot_v = (img_width - 1) - u; 
            }
            float final_x, final_y;
            // 2. Handle Crop Offset and Dual-Image placement
            // Where is this pixel in the ACTUAL raw dual-fisheye image?
            if (front_mask_.at<uchar>(y, x)) {
                // Front is usually the right half of the raw image (check your camera!)
                final_x = img_width + x_offset_crop + rot_u; 
                final_y = y_offset_crop + rot_v;
            } else {
                // Back is the left half
                final_x = x_offset_crop + rot_u;
                final_y = y_offset_crop + rot_v;
            }
            
            
            
            // Write to the unified map
            full_map_x_.at<float>(y, x) = final_x;
            full_map_y_.at<float>(y, x) = final_y;

        }
    }
    
   

    maps_initialized_ = true;
    
    RCLCPP_INFO(get_logger(), "Mapping matrices initialization complete");
}


void EquirectangularNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr dual_fisheye_msg)
{
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(dual_fisheye_msg, "rgb8");
        int rows_size=cv_ptr->image.rows;
        int cols_size=cv_ptr->image.cols/2;
        if (!maps_initialized_ || params_changed_ ||
                rows_size != img_height_ || cols_size != img_width_) {
                // Pass the full raw image dimensions
                initMapping(rows_size, cols_size);
                params_changed_ = false;
            }
        auto start_time = now();
        // 3. Single Remap (Directly from Raw to Equirectangular)
        cv::remap(cv_ptr->image, equirect_img, full_map_x_, full_map_y_, cv::INTER_LINEAR);
        
        
        //cv::Mat equirect_img = createEquirectangular(front_img, back_img);
        
        // Publish result
        cv_bridge::CvImage out_msg;
        out_msg.header = dual_fisheye_msg->header;
        out_msg.encoding = "rgb8";
        out_msg.image = equirect_img;
        equirect_pub_->publish(*out_msg.toImageMsg());
        
        auto process_time = (now() - start_time).seconds();
        RCLCPP_DEBUG(get_logger(), "Processing time: %.3f seconds", process_time);
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error processing images: %s", e.what());
    }
}

rcl_interfaces::msg::SetParametersResult EquirectangularNode::parametersCallback(
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
    
    auto node = std::make_shared<EquirectangularNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
