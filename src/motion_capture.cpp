#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

class RedCircleDetector : public rclcpp::Node {
public:
    RedCircleDetector() : Node("red_circle_detector") {
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw_filtered", 10,
            std::bind(&RedCircleDetector::colorImageCallback, this, std::placeholders::_1)
        );

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/aligned_depth_to_color/image_raw", 10,
            std::bind(&RedCircleDetector::depthImageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/aligned_depth_to_color/camera_info", 10,
            std::bind(&RedCircleDetector::cameraInfoCallback, this, std::placeholders::_1)
        );

        point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/marker/position", 10);

        intrinsics_initialized_ = false;
        alpha_ = 0.1;
        prev_point_.x = 0.0;
        prev_point_.y = 0.0;
        prev_point_.z = 0.0;
    }

private:
    void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
            cv::Moments m = cv::moments(gray_image, true);
            if (m.m00 != 0) {
                cX_ = static_cast<int>(m.m10 / m.m00);
                cY_ = static_cast<int>(m.m01 / m.m00);
                circle_detected_ = true;
            } else {
                circle_detected_ = false;
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!circle_detected_ || !intrinsics_initialized_) return;
        
        try {
            cv::Mat depth_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
            uint16_t depth_value = depth_image.at<uint16_t>(cY_, cX_);
            float pixel[2] = {static_cast<float>(cX_), static_cast<float>(cY_)};
            float point[3];
            rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, depth_value * depth_scale_);
            geometry_msgs::msg::Point point_msg;
            point[0] = point[0] * 100.0;
            point[1] = point[1] * 100.0;
            point[2] = point[2] * 100.0;
            point[0] = alpha_ * point[0] + (1 - alpha_) * prev_point_.x;
            point[1] = alpha_ * point[1] + (1 - alpha_) * prev_point_.y;
            point[2] = alpha_ * point[2] + (1 - alpha_) * prev_point_.z;
            prev_point_.x = point[0];
            prev_point_.y = point[1];
            prev_point_.z = point[2];
            point_msg.x = point[0];
            point_msg.y = point[1];
            point_msg.z = point[2];
            point_pub_->publish(point_msg);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (intrinsics_initialized_) return;
        intrinsics_.width = msg->width;
        intrinsics_.height = msg->height;
        intrinsics_.ppx = msg->k[2];
        intrinsics_.ppy = msg->k[5];
        intrinsics_.fx = msg->k[0];
        intrinsics_.fy = msg->k[4];
        intrinsics_.model = (msg->distortion_model == "plumb_bob") ?
                            RS2_DISTORTION_BROWN_CONRADY : RS2_DISTORTION_KANNALA_BRANDT4;
        for (int i = 0; i < 5; ++i) intrinsics_.coeffs[i] = msg->d[i];
        depth_scale_ = 0.001;
        intrinsics_initialized_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rs2_intrinsics intrinsics_;
    bool intrinsics_initialized_;
    bool circle_detected_;
    int cX_, cY_;
    float depth_scale_;
    geometry_msgs::msg::Point prev_point_;
    float alpha_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedCircleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
