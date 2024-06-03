#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <iostream>

class CameraInterface : public rclcpp::Node {
public:
    CameraInterface() : Node("camera_interface_node") {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraInterface::captureAndPublish, this)
        );

        pipeline_ = "v4l2src device=/dev/video11 ! video/x-raw,format=NV12,width=800,height=600,framerate=30/1 ! videoconvert ! appsink";
        cap_.open(pipeline_, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera stream.");
        }
    }

private:
    void captureAndPublish() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty frame.");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string pipeline_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInterface>());
    rclcpp::shutdown();
    return 0;
}