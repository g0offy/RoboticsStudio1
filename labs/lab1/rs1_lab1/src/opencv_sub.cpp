#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp> 


class OpenCVSub : public rclcpp::Node {
public:
    OpenCVSub() : Node("open_cv_sub") {
        // Subscription to the camera topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&OpenCVSub::callback, this, std::placeholders::_1));
        // Publisher to the modified image topic
        // pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera_modified", 10);
        pub_ =  this->create_publisher<sensor_msgs::msg::Image>("camera_modified", 10);
    }

private:
    // this function will accept the sub_ msg, convert it from ROS message to openCV, draw an image, , convert it back, then publish it back
    void callback(const sensor_msgs::msg::Image::SharedPtr msg){
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        //drawing circle
        cv::Point center(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2);
        int radius = 50; // You can adjust the radius as needed
        cv::Scalar color(0, 255, 0); // Green circle in BGR format
        int thickness = 2; // Thickness of the circle border

        cv::circle(cv_ptr->image, center, radius, color, thickness);

        // Convert the modified OpenCV image back to a ROS image message
        sensor_msgs::msg::Image::SharedPtr modified_msg = cv_ptr->toImageMsg();

        // Publish the modified image
        pub_->publish(*modified_msg);
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenCVSub>());
    rclcpp::shutdown();
    return 0;
}
