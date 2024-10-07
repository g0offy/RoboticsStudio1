#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class ImageOverlayNode : public rclcpp::Node
{
public:
    ImageOverlayNode() : Node("image_overlay_node")
    {
        overlay_images();
    }

private:
    void overlay_images()
    {
        // Load the images
        cv::Mat image1 = cv::imread("/home/krel/ros2_ws/src/sprint3/share/libraryV3_map.pgm", cv::IMREAD_GRAYSCALE);
        cv::Mat image2 = cv::imread("/home/krel/ros2_ws/src/sprint3/share/map.pgm", cv::IMREAD_GRAYSCALE);

        if (image1.empty() || image2.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the images!");
            return;
        }

        // Function to find the bounding box of the non-white regions
        auto find_bounding_box = [](const cv::Mat& image) -> cv::Rect {
            cv::Mat binary;
            cv::threshold(image, binary, 254, 255, cv::THRESH_BINARY_INV);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            return cv::boundingRect(contours[0]);
        };

        // Find bounding boxes
        cv::Rect bbox1 = find_bounding_box(image1);
        cv::Rect bbox2 = find_bounding_box(image2);

        // Crop images to their bounding boxes
        cv::Mat cropped_image1 = image1(bbox1);
        cv::Mat cropped_image2 = image2(bbox2);

        // Display the cropped images
        cv::imshow("Cropped Image 1", cropped_image1);
        cv::imshow("Cropped Image 2", cropped_image2);

        // Check if the cropped images have the same size
        if (cropped_image1.size() != cropped_image2.size())
        {
            RCLCPP_WARN(this->get_logger(), "Cropped images are not the same size! Resizing image2 to match image1.");
            cv::resize(cropped_image2, cropped_image2, cropped_image1.size());
        }

        // Overlay the images
        cv::Mat overlay;
        cv::addWeighted(cropped_image1, 0.5, cropped_image2, 0.5, 0.0, overlay);

        // Display the result
        cv::imshow("Overlay Image", overlay);
        cv::waitKey(0); // Wait for a key press to close the windows

        RCLCPP_INFO(this->get_logger(), "Overlay image displayed successfully.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageOverlayNode>());
    rclcpp::shutdown();
    return 0;
}