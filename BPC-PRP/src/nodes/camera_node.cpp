//
// Created by [Tvoje Jméno] on [Dnešní Datum].
//

#include "nodes/camera_node.hpp"

namespace nodes {

    CameraNode::CameraNode() : rclcpp::Node("camera_node") {
        detector_ = std::make_unique<algorithms::ArucoDetector>();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            10,
            std::bind(&CameraNode::image_callback, this, std::placeholders::_1)
        );

        image_pub_ = image_transport::create_publisher(this, "camera/detected_markers");

        command_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/camera/command",
            rclcpp::QoS(10)
        );

        RCLCPP_INFO(this->get_logger(), "Camera Node: Posilam POUZE zmeny platnych markeru.");
    }

    void CameraNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        if (frame.empty()) {
            return;
        }

        last_detections_ = detector_->detect(frame);
        last_frame_ = frame.clone();

        // Interní stav - výchozí je 55 ("nic nevidim")
        uint8_t current_cmd = 55;

        if (!last_detections_.empty()) {
            for (const auto& marker : last_detections_) {
                std::vector<std::vector<cv::Point2f>> corners = {marker.corners};
                std::vector<int> ids = {marker.id};
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
            }

            int detected_id = last_detections_[0].id;

            if (detected_id == 0 || detected_id == 1 || detected_id == 2 ||
                detected_id == 10 || detected_id == 11 || detected_id == 12) {

                current_cmd = static_cast<uint8_t>(detected_id);
            }
        }

        // Zkontrolujeme, jestli se stav změnil (např. z 55 na 1, nebo z 1 na 55)
        if (current_cmd != last_published_cmd_) {

            // KOUZLO: Publikujeme POUZE tehdy, pokud nový stav NENÍ 55
            if (current_cmd != 55) {
                std_msgs::msg::UInt8 cmd_msg;
                cmd_msg.data = current_cmd;
                command_pub_->publish(cmd_msg);

                RCLCPP_INFO(this->get_logger(), "Novy platny marker! Odesilam: %d", current_cmd);
            }

            // Paměť ale aktualizujeme VŽDY!
            // Tím zaručíme, že přechod "Marker -> Nic -> Stejný Marker" zafunguje správně.
            last_published_cmd_ = current_cmd;
        }

        // Publikování obrazu pro RViz2 zůstává stejné
        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        image_pub_.publish(out_msg);
    }

} // namespace nodes