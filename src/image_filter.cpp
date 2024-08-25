#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorFilterNode : public rclcpp::Node
{
public:
    ColorFilterNode()
        : Node("image_filter")
    {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ColorFilterNode::image_callback, this, std::placeholders::_1));

        filtered_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Scalar lower_red1(0, 100, 100);
        cv::Scalar upper_red1(10, 255, 255);

        cv::Scalar lower_red2(160, 100, 100);
        cv::Scalar upper_red2(179, 255, 255);

        cv::Mat mask1, mask2;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);

        cv::Mat mask = mask1 | mask2;

        cv::Mat filtered_image;
        cv::bitwise_and(image, image, filtered_image, mask);

        auto filtered_msg = cv_bridge::CvImage(msg->header, "bgr8", filtered_image).toImageMsg();
        filtered_image_publisher_->publish(*filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_image_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorFilterNode>());
    rclcpp::shutdown();
    return 0;
}
