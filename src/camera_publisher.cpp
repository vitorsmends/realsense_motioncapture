#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
        : Node("camera_publisher")
    {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_1/image_raw", 10);
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_1/camera_info", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CameraPublisher::publish_image, this));

        cap_.open("/dev/video2");

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
        }

        // Initialize camera info based on provided data
        camera_info_msg_.header.frame_id = "camera_1";
        camera_info_msg_.height = 480;
        camera_info_msg_.width = 640;
        camera_info_msg_.distortion_model = "plumb_bob";
        camera_info_msg_.d = {0, 0, 0, 0, 0};  // No distortion for simplicity

        // Example intrinsic matrix
        camera_info_msg_.k = {
            640, 0, 320,  // fx, 0, cx
            0, 640, 240,  // 0, fy, cy
            0, 0, 1       // 0, 0, 1
        };

        // Example rectification matrix (identity matrix)
        camera_info_msg_.r = {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

        // Example projection matrix
        camera_info_msg_.p = {
            640, 0, 320, 0,  // fx, 0, cx, 0
            0, 640, 240, 0,  // 0, fy, cy, 0
            0, 0, 1, 0       // 0, 0, 1, 0
        };
    }

private:
    void publish_image()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture image");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_1";
        image_publisher_->publish(*msg);

        // Publish camera info
        camera_info_msg_.header.stamp = msg->header.stamp;
        camera_info_publisher_->publish(camera_info_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
