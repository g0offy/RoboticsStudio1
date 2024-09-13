#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

        // Functionality:

        //     Convert LaserScan to Image: The laserScanToMat function is called to convert the LaserScan data into an image (a cv::Mat).
        //     Handle First Image: If it's the first time this function is being called, it captures and displays the first image.
        //     Handle Second Image: If it's the second time this function is called, it captures and displays the second image, then calculates the change in orientation (yaw).
        //     Update and Rotate: For subsequent calls, it updates the images, calculates the yaw change, and logs the relative orientation.

        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat img = laserScanToMat(msg);

        if (!first_image_captured_)
        {
            first_image_ = img.clone();
            first_image_captured_ = true;
            // Display the first image
            cv::imshow("First Image", first_image_);
            cv::waitKey(1); // Add this to process GUI events and update the window
            // Rotate the robot by publishing to cmd_vel
            rotateRobot();
        }
        else if (!second_image_captured_)
        {
            second_image_ = img.clone();
            second_image_captured_ = true;
            // Display the second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1); // Add this to process GUI events and update the window
            // Calculate the change in yaw using cv::transform
            calculateYawChange();
        }
        else
        {
            first_image_ = second_image_.clone();
            second_image_ = img.clone();
            // Display the new second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1); // Add this to process GUI events and update the window

            calculateYawChange();
            relative_orientaion_ = relative_orientaion_ + angle_difference_;
            // // Normalize the angle between -180 and 180 degrees
            // relative_orientaion_ = std::fmod(relative_orientaion_, 360.0);
            if (relative_orientaion_ > 180.0)
            {
                relative_orientaion_ -= 360.0;
            }
            else if (relative_orientaion_ < -180.0)
            {
                relative_orientaion_ += 360.0;
            }
            RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
            // rotate the turtlebot to the correct orientation, try to bring the relative orientation to 0
        }
        // Use a P controller to rotate the robot and bring the relative orientation to 0
        if (relative_orientaion_ > 0.005)
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = 0.05; // Rotate clockwise with angular velocity of 0.5
            cmd_publisher_->publish(twist_msg);
        }
        else if (relative_orientaion_ < -0.005)
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = -0.05; // Rotate anti-clockwise with angular velocity of 0.5
            cmd_publisher_->publish(twist_msg);
        }
        else
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = 0.0; // Stop rotation
            cmd_publisher_->publish(twist_msg);
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {

        // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

        // Functionality:

        //      Create Image: Initializes a blank image of size 500x500 pixels.
        //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.

        // Parameters
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) * img_size / (2 * max_range)) + img_size / 2);
                int y = static_cast<int>((range * sin(angle) * img_size / (2 * max_range)) + img_size / 2);
                if (x >= 0 && x < img_size && y >= 0 && y < img_size)
                {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void rotateRobot()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 1.0; // Rotate with some angular velocity
        cmd_publisher_->publish(twist_msg);

        // Sleep for a while to allow the robot to rotate
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    void calculateYawChange()
    {
        // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

        // Functionality:

        //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
        // std::vector<cv::Point2f> srcPoints, dstPoints;
        // detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
        //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
        //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.

        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try
        {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            }
            else
            {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = std::fmod(angle_difference_, 360.0);
                if (angle_difference_ < 0)
                {
                    angle_difference_ += 360.0;
                }
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}