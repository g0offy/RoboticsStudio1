#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Define the DetectCylinderNode class, inheriting from rclcpp::Node
class DetectCylinderNode : public rclcpp::Node
{
private:
    // Subscription to the laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    // Struct to represent a point in the laser scan data
    struct Point
    {
        float x;     // X coordinate
        float y;     // Y coordinate
        float range; // Range value

        // Parameterized constructor
        Point(float x, float y, float range) : x(x), y(y), range(range) {}
    };

    // Vector to store the cluster of points from laser scan data
    std::vector<Point> clusterVec;

    // Parameters for circle detection
    const double cylinder_radius = 0.3; // 30 cm
    const double tolerance = 0.05;      // 5 cm tolerance

    // Temporary image for processing
    cv::Mat tempImage;

public:
    // ======================== Class Members ======================== //

    /**
     * @brief Construct a new Detect Cylinder Node object
     */
    DetectCylinderNode() : Node("detect_cylinder_node")
    {
        RCLCPP_INFO(this->get_logger(), "Running Detect Cylinder Node");

        // Create a subscription to the laser scan data
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DetectCylinderNode::scanCallback, this, std::placeholders::_1));

        // Create OpenCV windows to display various images
        cv::namedWindow("LaserScan", cv::WINDOW_AUTOSIZE);        // Window to display laser readings
        cv::namedWindow("CylinderDetected", cv::WINDOW_AUTOSIZE); // Window to display detected cylinders
        cv::namedWindow("GrayImage", cv::WINDOW_AUTOSIZE);        // Window to display grayscale image
        cv::namedWindow("EdgesImage", cv::WINDOW_AUTOSIZE);       // Window to display edges image
    }

    /**
     * @brief Destroy the Detect Cylinder Node object
     */
    ~DetectCylinderNode()
    {
    }

    /**
     * @brief This function is called whenever a new laser scan message is received
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if the laser scan data is empty
        if (msg->ranges.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty laser scan data.");
            return;
        }

        // Display the laser scan data in a window
        tempImage = displayLaserScan(msg);

        // Detect cylinders in the laser scan data using OpenCV
        detectCylinderOpenCv(tempImage);

        // Find clusters in the laser scan data
        clusterVec = findCluster(msg);
    }

    /**
     * @brief This function displays the laser scan data in an OpenCV window
     * @param scan The laser scan message
     * @return The image with the laser scan data
     */
    cv::Mat displayLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int img_size = 800; // Size of the image
        float max_range = scan->range_max; // Maximum range of the laser scan

        // Create a black image (all values initialized to 0)
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        // Iterate through the laser scan ranges
        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];
            // Check if the range is within the valid range
            if (range > scan->range_min && range < scan->range_max)
            {
                // Calculate x and y based on the range and angle of the scan
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) * img_size / (2 * max_range)) + img_size / 2);
                int y = static_cast<int>((range * sin(angle) * img_size / (2 * max_range)) + img_size / 2);

                // Ensure the calculated point is within the image bounds
                if (x >= 0 && x < img_size && y >= 0 && y < img_size)
                {
                    // Assign brightness based on proximity: closer points are brighter
                    int intensity = static_cast<int>((1 - range / max_range) * 255 * 5);
                    image.at<uchar>(y, x) = 255 * 2; // Set pixel brightness
                }
            }
        }

        // Display the image in the OpenCV window
        cv::imshow("LaserScan", image);
        cv::waitKey(1); // Wait for a key press (1 ms)
        return image; // Return the image with the laser scan data
    }

    /**
     * @brief This function detects a cylinder given a set of laser data points
     */
    void detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Placeholder function for cylinder detection
    }

    /**
     * @brief This function detects a cylinder using OpenCV
     * @param image The image to detect the cylinder in
     */
    void detectCylinderOpenCv(cv::Mat &image)
    {
        // Check if the image is empty
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty image, skipping cylinder detection.");
            return;
        }

        // Convert the image to grayscale
        cv::Mat gray;
        if (image.channels() == 3 || image.channels() == 4)
        {
            // Convert to grayscale if the image has 3 or 4 channels
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }
        else if (image.channels() == 1)
        {
            // Image is already in grayscale
            gray = image;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels in input image.");
            return;
        }

        // Apply Canny edge detector
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);

        // Apply Gaussian blur to reduce noise and improve circle detection
        cv::GaussianBlur(edges, gray, cv::Size(9, 9), 2, 2);

        // Apply Hough Circle Transform to detect circles
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 0.9, 15, 30, 10, 14, 16);

        // Draw the detected circles
        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // Draw circle center
            cv::drawMarker(image, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 1, 8);
            // Draw circle perimeter
            cv::circle(image, center, c[2], cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        }

        // Display the result in OpenCV windows
        cv::imshow("CylinderDetected", image);
        // cv::imshow("GrayImage", gray);
        // cv::imshow("EdgesImage", edges);

        cv::waitKey(3); // Wait for a key press (3 ms)
    }

    /**
     * @brief This function finds clusters of points in the laser scan data
     * @param msg The laser scan message
     * @return A vector of points representing the clusters found
     */
    std::vector<Point> findCluster(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if the laser scan data is empty
        if (msg->ranges.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty laser scan data. Skipping cluster detection.");
            return {};
        }

        // Vector to store the cluster of points from laser scan data
        std::vector<Point> clustersfound;

        // Extract ranges and angles from the laser scan message
        std::vector<float> ranges = msg->ranges;
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        double angle_max = msg->angle_max;

        // Iterate through the ranges to find clusters
        for (std::size_t i = 0; i < ranges.size(); ++i)
        {
            // Check if the range value is finite and within the valid range
            if (std::isfinite(ranges[i]) && ranges[i] < msg->range_max && ranges[i] > msg->range_min)
            {
                // Collect points in a cluster
                for (std::size_t j = i; j < ranges.size(); ++j)
                {
                    // Check if the range difference is within the tolerance
                    if (std::abs(ranges[j] - ranges[i]) < tolerance)
                    {
                        // Calculate x and y based on the range and angle
                        double angle = angle_min + j * angle_increment;
                        float x = ranges[j] * std::cos(angle);
                        float y = ranges[j] * std::sin(angle);
                        clustersfound.emplace_back(x, y, ranges[j]);
                    }
                    else
                    {
                        break; // Break the inner loop if the range difference exceeds the tolerance
                    }
                }
            }
        }

        return clustersfound; // Return the clusters found
    }
};

// Main function
int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create and spin the DetectCylinderNode
    rclcpp::spin(std::make_shared<DetectCylinderNode>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}