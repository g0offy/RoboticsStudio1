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
        
        cv::imshow(WINDOW3, scan_image);
        cv::waitKey(1);

        if (!map_image_.empty()) {
            // Capture the first image (map image) from the map callback
            scan_image_ = scan_image.clone();
            calculateYawChange();
            visualizeMatches(map_section_, scan_image_);
            cv::waitKey(1);
        }
    }
    

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat map_image = m_MapColImage.clone();

        double roi_width = 65;  // Set desired width for the ROI
        double roi_height = 100;  // Set desired height for the ROI
        // Extract section of the map around the robot (Image A)
        double roi_x = std::max(0.0, std::min(robot_x_ - roi_width / 2,static_cast<double>(map_image.cols) - roi_width));
        double roi_y = std::max(0.0, std::min(robot_y_ - roi_height / 2, static_cast<double>(map_image.rows) - roi_height));

        // Ensure ROI is within bounds
        if (roi_width > 0 && roi_height > 0) {
            cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
            cv::Mat map_section = map_image(roi).clone(); // Extracted section of the map

            // Rotate Image A based on the robot's orientation
            cv::Mat map_section_rotated;
            cv::Point2f center(map_section.cols / 2.0, map_section.rows / 2.0);
            cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, robot_theta_ * 180.0 / CV_PI, 1.0);
            cv::warpAffine(map_section, map_section_rotated, rotation_matrix, map_section.size());

            // Resize Image A to make it larger
            cv::Mat map_section_resized;
            cv::resize(map_section_rotated, map_section_resized, cv::Size(), 3.0, 3.0, cv::INTER_LINEAR);
            // resize to match scan
            // cv::resize(map_section_rotated, map_section_resized, scan_image_.size(), 0, 0, cv::INTER_LINEAR);

            map_section_ = map_section_resized.clone();
            // // Display the resized section for verification
            cv::imshow(WINDOW2, map_section_);
        }

        cv::rotate(map_image, map_image, cv::ROTATE_90_COUNTERCLOCKWISE);


        cv::imshow(WINDOW1, map_image);

        map_image_ = map_image.clone();

        cv::waitKey(1);

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_theta_ = relative_orientaion_;
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

    void calculateYawChange() {

        std::vector<cv::Point2f> mapPoints, scanPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;
        detectAndMatchFeatures(map_section_, scan_image_, mapPoints, scanPoints, keypoints1, keypoints2, goodMatches);

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

    void filterMatchesByHomography(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, 
                               std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& filtered_matches) 
    {
        if (matches.size() < 4) return;  // Need at least 4 matches to compute homography

        // Convert keypoints to Point2f
        std::vector<cv::Point2f> points1, points2;
        for (const auto& match : matches) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        // Use RANSAC to find homography and remove outliers
        std::vector<uchar> inliersMask(points1.size());
        cv::Mat homography = cv::findHomography(points1, points2, cv::RANSAC, 3.0, inliersMask);

        for (size_t i = 0; i < inliersMask.size(); i++) {
            if (inliersMask[i]) {
                filtered_matches.push_back(matches[i]);
            }
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints,
                                std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2,
                                std::vector<cv::DMatch>& goodMatches) 
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
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

        // // Determine the number of top matches to keep (10% of total matches)
        // size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.10);

        // // Keep only the best matches (top 15%)
        // goodMatches.assign(matches.begin(), matches.begin() + numGoodMatches);
        // Keep only the best matches (top 10%)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.10);
        matches.resize(numGoodMatches);

        // Filter matches using homography
        filterMatchesByHomography(keypoints1, keypoints2, matches, goodMatches);


        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    void visualizeMatches(const cv::Mat& img1, const cv::Mat& img2) {
        // Detect and match features between Image B and Image C
        std::vector<cv::Point2f> mapPoints, scanPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;
        detectAndMatchFeatures(img1, img2, mapPoints, scanPoints, keypoints1, keypoints2, goodMatches);

        // Draw matches
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches);

        // Display the matches
        cv::imshow(WINDOW4, img_matches);
        cv::waitKey(1);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    
    cv::Mat scan_image_;
    cv::Mat map_image_;
    cv::Mat map_section_;

    std::vector<cv::DMatch> matches_;

    bool scan_image_captured_ = false;
    bool map_image_captured_ = false;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;

    double roi_size_ = 50;

    cv::Point robot_position_;
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
    const std::string WINDOW2 = "Image: Map section";
    const std::string WINDOW3 = "Image: Laser Scan";
    const std::string WINDOW4 = "Image: Feature detection";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

