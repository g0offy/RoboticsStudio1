#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanModifier : public rclcpp::Node
{
public:
    LaserScanModifier()
    : Node("laser_scan_modifier")
    {
        // Subscription to /scan topic
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanModifier::callback, this, std::placeholders::_1));
        // Publisher created for /filtered_scan topic
        pub1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("modified_scan", 10);
        // Step size for filtering
        n_ = 5; // filtered by nth value, displays every nth point
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        auto modifiedMsg = sensor_msgs::msg::LaserScan();
        modifiedMsg.header = msg->header;
        modifiedMsg.angle_min = msg->angle_min;
        modifiedMsg.angle_max = msg->angle_max;
        modifiedMsg.angle_increment = msg->angle_increment * n_;
        modifiedMsg.time_increment = msg->time_increment * n_;
        modifiedMsg.scan_time = msg->scan_time;
        modifiedMsg.range_min = msg->range_min;
        modifiedMsg.range_max = msg->range_max;
        // Filter ranges
        for (size_t i = 0; i < msg->ranges.size(); i += n_) {
            modifiedMsg.ranges.push_back(msg->ranges[i]);
        }
        // Publish the filtered message
        pub1_->publish(modifiedMsg);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub1_;
    size_t n_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanModifier>());
    rclcpp::shutdown();
    return 0;
}