#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class overlayer : public rclcpp::Node
{
public:
    overlayer() : Node("overlayer")
    {
        overlay_images();
    }

private:
    void overlay_images()
    {
        // Load the images
        cv::Mat image1 = cv::imread("/home/student/map.pgm", cv::IMREAD_GRAYSCALE);
        cv::Mat image2 = cv::imread("/home/student/ros2_ws/my_map.pgm", cv::IMREAD_GRAYSCALE);

        if (image1.empty() || image2.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the images!");
            return;
        }

        // Set desired size and position for image2 (my_map.pgm)
        double scaleFactor = 5.1; // Scaling factor for image2
        int x = 74; // X-coordinate for placing image2
        int y = 107; // Y-coordinate for placing image2
        double alpha = 0.5; // Transparency for image2

        // Scale image2
        cv::Mat scaled_image2;
        cv::resize(image2, scaled_image2, cv::Size(), scaleFactor, scaleFactor);

        // Display the scaled image2 in a separate window with transparency
        cv::Mat transparent_image2;
        cv::addWeighted(scaled_image2, alpha, cv::Mat(scaled_image2.size(), scaled_image2.type(), cv::Scalar(255)), 1 - alpha, 0.0, transparent_image2);
        cv::namedWindow("Scaled Image 2 (Transparent)", cv::WINDOW_NORMAL);
        cv::resizeWindow("Scaled Image 2 (Transparent)", 800, 600);
        cv::imshow("Scaled Image 2 (Transparent)", transparent_image2);

        // Display image1 in a separate window
        cv::namedWindow("Image 1", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 1", 800, 600);
        cv::imshow("Image 1", image1);

        // Create a white canvas large enough to contain both images
        int canvas_width = std::max(image1.cols, x + scaled_image2.cols);
        int canvas_height = std::max(image1.rows, y + scaled_image2.rows);
        cv::Mat canvas(canvas_height, canvas_width, CV_8UC1, cv::Scalar(255));

        // Place image1 on the canvas at the origin
        image1.copyTo(canvas(cv::Rect(0, 0, image1.cols, image1.rows)));

        // Blend the scaled image2 onto the canvas at the specified (x, y) coordinates with transparency
        cv::Mat roi = canvas(cv::Rect(x, y, scaled_image2.cols, scaled_image2.rows));
        cv::addWeighted(scaled_image2, alpha, roi, 1.0 - alpha, 0.0, roi);

        // Display the final overlay result in a smaller window
        std::string windowName = "Overlay on Canvas";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowName, 800, 600);
        cv::imshow(windowName, canvas);

        // Wait for a key press to close all windows
        cv::waitKey(0);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<overlayer>());
    rclcpp::shutdown();
    return 0;
}
