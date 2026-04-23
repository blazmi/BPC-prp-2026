#include "../include/nodes/Line_node.hpp"
#include <algorithm>
#include <std_msgs/msg/bool.hpp>

namespace nodes {

    LineNode::LineNode() : Node("line_node")
    {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 10,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/line_node/continuous_pose", 10
        );

        // NOVÉ: Publisher pro detekci kolmé čáry (hranice buňky)
        crossline_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/line_node/crossline", 10
        );

        calibrate_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/robot/calibrate", 10,
            std::bind(&LineNode::calibrate_callback, this, std::placeholders::_1)
        );
    }

    LineNode::~LineNode() {}

    void LineNode::calibrate_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        is_calibrating_ = true;
        calibration_start_time_ = this->now();

        min_l_ = 65535.0f; max_l_ = 0.0f;
        min_r_ = 65535.0f; max_r_ = 0.0f;

        RCLCPP_INFO(this->get_logger(), "KALIBRACE START: Hledam minima a maxima...");
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        uint16_t left = msg->data[0];
        uint16_t right = msg->data[1];

        if (is_calibrating_) {
            if ((this->now() - calibration_start_time_).seconds() > 5.0) {
                is_calibrating_ = false;
                RCLCPP_INFO(this->get_logger(), "KALIBRACE HOTOVA! L(min:%.0f max:%.0f) R(min:%.0f max:%.0f)",
                            min_l_, max_l_, min_r_, max_r_);
            } else {
                if (left < min_l_) min_l_ = left;
                if (left > max_l_) max_l_ = left;
                if (right < min_r_) min_r_ = right;
                if (right > max_r_) max_r_ = right;
                return;
            }
        }

        float range_l = (max_l_ - min_l_ > 0.0f) ? (max_l_ - min_l_) : 1.0f;
        float range_r = (max_r_ - min_r_ > 0.0f) ? (max_r_ - min_r_) : 1.0f;

        float l_raw = (left - min_l_) / range_l;
        float r_raw = (right - min_r_) / range_r;

        float l_calibrated = std::clamp(l_raw, 0.0f, 1.0f);
        float r_calibrated = std::clamp(r_raw, 0.0f, 1.0f);

        // ==========================================
        // DETEKCE ČÁRY: Pokud oba senzory vidí silně černou (> 0.6)
        // ==========================================
        bool on_crossline = (l_calibrated > 0.6f && r_calibrated > 0.6f);
        std_msgs::msg::Bool cross_msg;
        cross_msg.data = on_crossline;
        crossline_pub_->publish(cross_msg);

        float current_continuous = algorithms::LineEstimator::estimate_continuous(l_calibrated, r_calibrated);
        continuous_.store(current_continuous);
        discrete_.store(algorithms::LineEstimator::estimate_discrete(l_calibrated, r_calibrated));

        std_msgs::msg::Float32 pose_msg;
        pose_msg.data = current_continuous;
        pose_pub_->publish(pose_msg);
    }

} // namespace nodes