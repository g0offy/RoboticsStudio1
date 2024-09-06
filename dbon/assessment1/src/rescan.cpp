#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber()
    : Node("laser_scan_subscriber")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanSubscriber::callback_Function, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);

        n_ = 10;
    }

private:
    void callback_Function(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        auto newMsg = sensor_msgs::msg::LaserScan();
        newMsg.header = msg->header;
        newMsg.angle_min = msg->angle_min;
        newMsg.angle_max = msg->angle_max;
        newMsg.angle_increment = msg->angle_increment * n_;
        newMsg.time_increment = msg->time_increment * n_;
        newMsg.scan_time = msg->scan_time;
        newMsg.range_min = msg->range_min;
        newMsg.range_max = msg->range_max;

        for (size_t j = 0; j < msg->ranges.size(); j += n_) {
            newMsg.ranges.push_back(msg->ranges[j]);
        }
        
        pub_->publish(newMsg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    size_t n_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSubscriber>());
    rclcpp::shutdown();
    return 0;
}