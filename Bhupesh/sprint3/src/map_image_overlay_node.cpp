#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class MapImageOverlayNode : public rclcpp::Node
{

public:
    MapImageOverlayNode() : Node("map_image_overlay_node")
    {
        RCLCPP_INFO(this->get_logger(), "Running MapImageOverlayNode");
        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
        cv::waitKey(2);

        ImageOverlay();
    }

    ~MapImageOverlayNode()
    {
        cv::destroyWindow("Display window");
    }

    void ImageOverlay()
    {
        RCLCPP_INFO(this->get_logger(), "Running ImageOverlay");

        ground_truth_image = cv::imread("/home/uts/git/RoboticsStudio1/Bhupesh/sprint3/share/libraryV3GroundTruth.pgm", cv::IMREAD_GRAYSCALE);
        slamtool_box_image = cv::imread("/home/uts/git/RoboticsStudio1/Bhupesh/sprint3/share/libraryV3_map.pgm", cv::IMREAD_GRAYSCALE);

        if (ground_truth_image.empty() || slamtool_box_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not read one of the images");
            return;
        }

        // crop the images
        cropped_ground_truth_image = CropWhiteSpace(ground_truth_image);
        cropped_slamtool_box_image = CropWhiteSpace(slamtool_box_image);

        // Resize the images to the same size
        if (cropped_ground_truth_image.size() != cropped_slamtool_box_image.size())
        {
            cv::resize(cropped_ground_truth_image, cropped_ground_truth_image, cropped_slamtool_box_image.size());
        }

        cv::addWeighted(cropped_slamtool_box_image, 0.5,cropped_ground_truth_image , 0.5, 0, overlayed_image);

        cv::imshow("Display window", overlayed_image);
        cv::waitKey(1);
    }

    cv::Mat CropWhiteSpace(cv::Mat &img)
    {
        cv::Mat binary_image;
        cv::threshold(img, binary_image, 254, 255, cv::THRESH_BINARY_INV);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Get bounding box of the largest contour
        cv::Rect bounding_box;
        if (!contours.empty())
        {
            bounding_box = cv::boundingRect(contours[0]);
            for (size_t i = 1; i < contours.size(); ++i)
            {
                bounding_box |= cv::boundingRect(contours[i]);
            }
        }

        // Crop the image
        cv::Mat cropped_img = img(bounding_box);

        return cropped_img;
    }

private:
    cv::Mat ground_truth_image;
    cv::Mat slamtool_box_image;
    cv::Mat cropped_ground_truth_image;
    cv::Mat cropped_slamtool_box_image;

    cv::Mat overlayed_image;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<MapImageOverlayNode>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}