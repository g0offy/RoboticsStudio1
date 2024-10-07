#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <opencv2/opencv.hpp>

class IdentifyNode : public rclcpp::Node
{
public:
    IdentifyNode() : Node("identify_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&IdentifyNode::laserCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&IdentifyNode::mapCallback, this, std::placeholders::_1));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const double known_diameter = 0.30; // 30 cm
        const double radius = known_diameter / 2.0;
        const double tolerance = 0.10; // Increased tolerance to 10 cm

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            double x = msg->ranges[i] * cos(angle);
            double y = msg->ranges[i] * sin(angle);

            // Debugging: Print the range and coordinates
            RCLCPP_INFO(this->get_logger(), "Range: %f, Coordinates: (%f, %f)", msg->ranges[i], x, y);

            // Check for cylindrical shape
            if (fabs(msg->ranges[i] - radius) < tolerance)
            {
                RCLCPP_INFO(this->get_logger(), "Cylindrical object detected at (%f, %f)", x, y);
                publishMarker(x, y);
                drawOnMap(x, y);
                break;
            }
        }
    }

    void publishMarker(double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = this->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.30;
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0; // Changed to red for better visibility
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Convert the map data to an OpenCV cv::Mat
        int width = msg->info.width;
        int height = msg->info.height;
        cv::Mat map_image(height, width, CV_8UC1);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = x + y * width;
                int value = msg->data[index];
                if (value == -1)
                {
                    map_image.at<uchar>(y, x) = 127; // Unknown
                }
                else
                {
                    map_image.at<uchar>(y, x) = static_cast<uchar>(value * 255 / 100); // Occupied or free
                }
            }
        }

        // Store the map image
        map_image_ = map_image;
    }

    void drawOnMap(double x, double y)
    {
        if (map_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Map image is not available yet!");
            return;
        }

        // Convert coordinates from robot's frame to map frame
        // Assuming the map frame is aligned with the robot's frame
        int img_x = static_cast<int>((x - map_origin_x_) / map_resolution_);
        int img_y = static_cast<int>((y - map_origin_y_) / map_resolution_);

        // Draw the marker on the map
        cv::circle(map_image_, cv::Point(img_x, img_y), 5, cv::Scalar(0, 0, 255), -1); // Red circle

        // Display the map with the marker
        cv::imshow("Map with Marker", map_image_);
        cv::waitKey(0); // Wait for a key press to close the window
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    cv::Mat map_image_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IdentifyNode>());
    rclcpp::shutdown();
    return 0;
}