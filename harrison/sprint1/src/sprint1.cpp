#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor(size_t n)
        : Node("laser_scan_processor"), n_(n)
    {
        // Subscribe to the /scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        // Publisher for the filtered scan data (will create new topic "/filtered_scan")
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
    {   

        auto filtered_scan = *scan;  // Create a copy of the original scan message

        // Adjust the increments to reflect the filtering
        filtered_scan.angle_increment = scan->angle_increment * n_;
        filtered_scan.time_increment = scan->time_increment * n_;

        // Resize the vectors to hold the filtered data
        filtered_scan.ranges.resize(scan->ranges.size() / n_);
        if (!scan->intensities.empty()) {
            filtered_scan.intensities.resize(scan->intensities.size() / n_);
        }

        // Populate the filtered data
        for (size_t i = 0, j = 0; i < scan->ranges.size(); i += n_, ++j) {
            filtered_scan.ranges[j] = scan->ranges[i];
            RCLCPP_INFO(this->get_logger(), "Adding range index: %zu", i);
        }

        // add intensity(colour) of the points
        if (!scan->intensities.empty()) {
            for (size_t i = 0, j = 0; i < scan->intensities.size(); i += n_, ++j) {
                filtered_scan.intensities[j] = scan->intensities[i];
                RCLCPP_INFO(this->get_logger(), "Adding intensity index: %zu", i);
            }
        }

        scan_pub_->publish(filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    size_t n_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    size_t n;
    std::cout << "Enter the value of n for filtering: ";
    std::cin >> n;

    auto node = std::make_shared<LaserScanProcessor>(n);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}