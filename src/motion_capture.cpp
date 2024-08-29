#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MarkerPositionNode : public rclcpp::Node
{
public:
    MarkerPositionNode(const std::string &topic_xz, const std::string &topic_y, double distance_xz, double distance_y)
        : Node("marker_position_node"), distance_xz_(distance_xz), distance_y_(distance_y)
    {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_xz, 10, std::bind(&MarkerPositionNode::image_callback, this, std::placeholders::_1));

        image_subscription_y_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_y, 10, std::bind(&MarkerPositionNode::image_callback_y, this, std::placeholders::_1));

        position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/marker/position", 10);

        RCLCPP_INFO(this->get_logger(), "Subscribed to XZ topic: %s", topic_xz.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribed to Y topic: %s", topic_y.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        int image_width = image.cols; // Largura da imagem em pixels

        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // Encontrar o maior contorno
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

            // Calcular a relação de pixels por cm usando a distância da câmera ao objeto
            double pixels_per_cm_xz = static_cast<double>(image_width) / (2.0 * distance_xz_);

            // Converter os valores de pixels para centímetros
            current_x_ = static_cast<double>(cx) / pixels_per_cm_xz;
            current_z_ = static_cast<double>(cy) / pixels_per_cm_xz;

            publish_position();
        }
    }

    void image_callback_y(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        int image_height = image.rows; // Altura da imagem em pixels

        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
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
            int cy = static_cast<int>(m.m01 / m.m00);

            double pixels_per_cm_y = static_cast<double>(image_height) / (2.0 * distance_y_);

            current_y_ = static_cast<double>(cy) / pixels_per_cm_y;

            publish_position();
        }
    }

    void publish_position()
    {
        if (current_x_ >= 0 && current_y_ >= 0 && current_z_ >= 0)
        {
            geometry_msgs::msg::Point position_msg;
            position_msg.x = current_x_;
            position_msg.y = current_y_;
            position_msg.z = current_z_;

            position_publisher_->publish(position_msg);

            current_x_ = -1;
            current_y_ = -1;
            current_z_ = -1;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_y_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;

    double current_x_ = -1;
    double current_y_ = -1;
    double current_z_ = -1;

    double distance_xz_;
    double distance_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("marker_position_node");

    std::string topic_xz = node->declare_parameter<std::string>("topic_xz", "/camera_1/image_raw_filtered");
    std::string topic_y = node->declare_parameter<std::string>("topic_y", "/camera/camera/color/image_raw_filtered");

    // Distâncias em cm entre as câmeras e o objeto
    double distance_xz = 57.0; // Distância da câmera XZ ao objeto
    double distance_y = 97.0;  // Distância da câmera Y ao objeto

    auto marker_position_node = std::make_shared<MarkerPositionNode>(topic_xz, topic_y, distance_xz, distance_y);
    rclcpp::spin(marker_position_node);
    rclcpp::shutdown();
    return 0;
}
