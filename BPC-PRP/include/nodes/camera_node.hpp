//
// Created by [Tvoje Jméno] on [Dnešní Datum].
//

#ifndef PRP_PROJECT_CAMERA_NODE_HPP
#define PRP_PROJECT_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp> // NOVÉ: Pro odesílání malých čísel (0-255)
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

#include "algorithms/aruco_detector.hpp"

namespace nodes {

    class CameraNode : public rclcpp::Node {
    public:
        CameraNode();
        ~CameraNode() override = default;

    private:
        void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        std::unique_ptr<algorithms::ArucoDetector> detector_;
        cv::Mat last_frame_;
        std::vector<algorithms::ArucoDetector::Aruco> last_detections_;

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
        image_transport::Publisher image_pub_;

        // Publisher nyní posílá jen jeden bajt
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr command_pub_;
        uint8_t last_published_cmd_ = 255;
    };

} // namespace nodes

#endif // PRP_PROJECT_CAMERA_NODE_HPP