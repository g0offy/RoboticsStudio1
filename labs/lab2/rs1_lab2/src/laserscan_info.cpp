#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanProcessor : public rclcpp::Node {
public:
    LaserScanProcessor() : Node("laser_scan_processor") {
        // Create a subscription to the laser scan topic
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserScanProcessor::laser_scan_callback, this, std::placeholders::_1));
        // Create a publisher for the modified laser scan data
        pub1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("modified_scan", 10);

    }

private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Define the angle range for the 60-degree cone in front of the robot
        float start_angle = (360 -30.0) * (M_PI / 180.0);
        float end_angle = 30.0 * (M_PI / 180.0);
    
        // Calculate the start and end indices
        int start_index = static_cast<int>((start_angle - msg->angle_min) / msg->angle_increment);
        int end_index = static_cast<int>((end_angle - msg->angle_min) / msg->angle_increment);
    
        // Ensure indices are within bounds
        start_index = std::max(0, start_index);
        end_index = std::min(static_cast<int>(msg->ranges.size()), end_index);
    
        // Create a new LaserScan message for the subset of range values
        auto modified_scan = sensor_msgs::msg::LaserScan();
        // Copy the header from the original message
        modified_scan.header = msg->header;
    
        // Set the angle range for the 30-degree cone
        modified_scan.angle_min = start_angle;
        modified_scan.angle_max = end_angle;
    
        // Copy the angle increment, time increment, and scan time from the original message
        modified_scan.angle_increment = msg->angle_increment;
        modified_scan.time_increment = msg->time_increment;
        modified_scan.scan_time = msg->scan_time;
    
        // Copy the range and intensity limits from the original message
        modified_scan.range_min = msg->range_min;
        modified_scan.range_max = msg->range_max;
    
        // Assign the range values within the specified angle range
        if (start_index <= end_index) {
            modified_scan.ranges.assign(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
            modified_scan.intensities.assign(msg->intensities.begin() + start_index, msg->intensities.begin() + end_index);
        } else {
            modified_scan.ranges.assign(msg->ranges.begin() + start_index, msg->ranges.end());
            modified_scan.ranges.insert(modified_scan.ranges.end(), msg->ranges.begin(), msg->ranges.begin() + end_index);
            modified_scan.intensities.assign(msg->intensities.begin() + start_index, msg->intensities.end());
            modified_scan.intensities.insert(modified_scan.intensities.end(), msg->intensities.begin(), msg->intensities.begin() + end_index);
        }
        // Publish the modified laser scan data
        pub1_->publish(modified_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub2_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
