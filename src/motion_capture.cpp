#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MarkerPositionNode : public rclcpp::Node
{
public:
    MarkerPositionNode()
        : Node("marker_position_node")
    {
        // Subscriptions
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/filtered_image", 10, std::bind(&MarkerPositionNode::image_callback, this, std::placeholders::_1));

        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10, std::bind(&MarkerPositionNode::depth_callback, this, std::placeholders::_1));

        color_camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10, std::bind(&MarkerPositionNode::color_camera_info_callback, this, std::placeholders::_1));

        depth_camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/depth/camera_info", 10, std::bind(&MarkerPositionNode::depth_camera_info_callback, this, std::placeholders::_1));

        position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/marker/position", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image message to an OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Convert to grayscale (only one channel is needed since the marker is isolated)
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        // Find contours in the binary image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // Find the largest contour
            int largest_contour_index = 0;
            double largest_area = 0;
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if (area > largest_area)
                {
                    largest_area = area;
                    largest_contour_index = i;
                }
            }

            cv::Moments m = cv::moments(contours[largest_contour_index]);
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);

            current_x_ = cx;
            current_y_ = cy;

            if (!current_depth_image_.empty())
            {
                publish_position();
            }
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat depth_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

        // Resize depth image to match color image dimensions if necessary
        if (!color_intrinsics_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for color camera intrinsics");
            return;
        }

        if (depth_image.size() != color_image_size_)
        {
            cv::resize(depth_image, depth_image, color_image_size_);
        }

        current_depth_image_ = depth_image;

        if (current_x_ >= 0 && current_y_ >= 0)
        {
            publish_position();
        }
    }

    void color_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        color_intrinsics_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data()));
        color_image_size_ = cv::Size(msg->width, msg->height);
        color_intrinsics_initialized_ = true;
    }

    void depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        depth_intrinsics_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data()));
        depth_intrinsics_initialized_ = true;
    }

    void publish_position()
    {
        // Verifica se as coordenadas estão dentro do limite da imagem de profundidade
        if (current_x_ < current_depth_image_.cols && current_y_ < current_depth_image_.rows)
        {
            uint16_t depth = current_depth_image_.at<uint16_t>(current_y_, current_x_);

            geometry_msgs::msg::Point position_msg;
            position_msg.x = static_cast<double>(current_x_);
            position_msg.y = static_cast<double>(current_y_);
            position_msg.z = static_cast<double>(depth) / 1000.0; // Convert depth to meters

            position_publisher_->publish(position_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Marker coordinates out of depth image bounds.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;

    cv::Mat current_depth_image_;
    cv::Mat color_intrinsics_, depth_intrinsics_;
    cv::Size color_image_size_;
    bool color_intrinsics_initialized_ = false;
    bool depth_intrinsics_initialized_ = false;
    int current_x_ = -1;
    int current_y_ = -1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPositionNode>());
    rclcpp::shutdown();
    return 0;
}
