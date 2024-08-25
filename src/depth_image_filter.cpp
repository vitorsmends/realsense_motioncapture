#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthImageFilterNode : public rclcpp::Node
{
public:
    DepthImageFilterNode()
        : Node("depth_image_filter_node")
    {
        depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10, std::bind(&DepthImageFilterNode::depth_image_callback, this, std::placeholders::_1));

        filtered_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_depth_image", 10);
    }

private:
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat depth_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

        cv::Mat depth_image_32f;
        depth_image.convertTo(depth_image_32f, CV_32F);

        cv::Mat filtered_depth_image_32f;
        cv::bilateralFilter(depth_image_32f, filtered_depth_image_32f, /*d=*/9, /*sigmaColor=*/75, /*sigmaSpace=*/75);

        cv::Mat filtered_depth_image;
        filtered_depth_image_32f.convertTo(filtered_depth_image, CV_16U);

        auto filtered_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1, filtered_depth_image).toImageMsg();
        filtered_depth_image_publisher_->publish(*filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_image_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImageFilterNode>());
    rclcpp::shutdown();
    return 0;
}
