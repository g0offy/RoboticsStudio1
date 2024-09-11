#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cmath>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node"), robot_x_(-2.0), robot_y_(-0.5), robot_theta_(0)
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            
        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE); // map from occupany grid
        cv::namedWindow(WINDOW2, cv::WINDOW_AUTOSIZE); // map section
        cv::namedWindow(WINDOW3, cv::WINDOW_AUTOSIZE); // edges
        cv::namedWindow(WINDOW4, cv::WINDOW_AUTOSIZE); // laser scan image
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::cout << "scanCallback" << std::endl;

        // Convert LaserScan to image
        cv::Mat scan_image = laserScanToMat(msg);
        

        cv::imshow(WINDOW4, scan_image);
        cv::waitKey(1);

        if (!map_image_.empty()) {
            // Capture the first image (map image) from the map callback
            scan_image_ = scan_image.clone();
            calculateYawChange();
            cv::waitKey(1);
        } 
    }
    

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat map_image = m_MapColImage.clone();

        // // Resize the map image to match the scan image size (500x500)
        // cv::resize(map_image_, map_image_, cv::Size(500, 500));

        cv::rotate(map_image, map_image, cv::ROTATE_90_COUNTERCLOCKWISE);

        // // Extract section of the map around the robot
        // cv::Mat map_section = extractMapSection(m_MapColImage, robot_x_, robot_y_, map_scale_, 100); // Image A
        
        // // Extract edges from Image A to create Image B
        // cv::Mat edges = extractEdges(map_section); // Image B

        cv::imshow(WINDOW1, map_image);
        // cv::imshow(WINDOW2, map_section);
        // cv::imshow(WINDOW3, edges);

        // map_image_ = edges.clone();
        map_image_ = map_image.clone();

        cv::waitKey(1);

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        // robot_theta_ = tf2::getYaw(msg->pose.pose.orientation); // Update robot's position and orientation
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
    {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) * img_size / (2 * max_range)) + img_size / 2);
                int y = static_cast<int>((range * sin(angle) * img_size / (2 * max_range)) + img_size / 2);
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                    0, 1, 0,
                                    0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image\n";

        // // Display the image to verify
        // cv::imshow("Occupancy Grid", m_MapColImage);
        // cv::waitKey(1);
    }
    cv::Mat extractMapSection(const cv::Mat& map_image, double robot_x, double robot_y, double map_resolution, int section_size)
    {
        // Convert robot coordinates from meters to pixels
        int robot_x_pixel = static_cast<int>((robot_x - origin_x) / map_resolution);
        int robot_y_pixel = static_cast<int>((robot_y - origin_y) / map_resolution);

        // Define the region of interest (ROI) around the robot
        int half_size = section_size / 2;
        cv::Rect roi(robot_x_pixel - half_size, robot_y_pixel - half_size, section_size, section_size);

        // Make sure the ROI is within the image bounds
        roi.x = std::max(0, roi.x);
        roi.y = std::max(0, roi.y);
        roi.width = std::min(map_image.cols - roi.x, roi.width);
        roi.height = std::min(map_image.rows - roi.y, roi.height);

        // Extract the section
        cv::Mat image_A = map_image(roi).clone();
        return image_A;
    }

    // Extract edges from an image
    cv::Mat extractEdges(const cv::Mat& image_A)
    {
        cv::Mat image_B;
        // Convert to grayscale if not already
        if (image_A.channels() > 1) {
            cv::cvtColor(image_A, image_A, cv::COLOR_BGR2GRAY);
        }

        // Apply Gaussian blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(image_A, blurred, cv::Size(5, 5), 1.5);

        // Apply Canny edge detector
        cv::Canny(blurred, image_B, 50, 150);

        return image_B;
    }

    void calculateYawChange() {

        std::vector<cv::Point2f> mapPoints, scanPoints;
        detectAndMatchFeatures(map_image_, scan_image_, mapPoints, scanPoints);

        if (mapPoints.size() < 3 || scanPoints.size() < 3){
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try{
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(mapPoints, scanPoints);
            if (transform_matrix.empty()){
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            }
            else{
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        }
        catch (const cv::Exception &e){
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& mapPoints, std::vector<cv::Point2f>& scanPoints) 
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
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            mapPoints.push_back(keypoints1[match.queryIdx].pt);
            scanPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    
    cv::Mat scan_image_;
    cv::Mat map_image_;

    bool scan_image_captured_ = false;
    bool map_image_captured_ = false;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;

    double robot_x_;
    double robot_y_;
    double robot_theta_;

    double map_scale_;
    double origin_x;
    double origin_y;

    unsigned int size_x;
    unsigned int size_y;
    double angle_difference_;
    double relative_orientaion_ = 0.0;

    const std::string WINDOW1 = "Image: Map";
    const std::string WINDOW2 = "Image A: Map section";
    const std::string WINDOW3 = "Image B: Map edges";
    const std::string WINDOW4 = "Image C: Laser Scan";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

