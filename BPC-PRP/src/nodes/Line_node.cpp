#include "../include/nodes/Line_node.hpp"
#include <algorithm>

namespace nodes {

    LineNode::LineNode() : Node("line_node")
    {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors",
            10,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/line_node/continuous_pose",
            10
        );

        // NOVÉ: Posloucháme povel ke kalibraci
        calibrate_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/robot/calibrate",
            10,
            std::bind(&LineNode::calibrate_callback, this, std::placeholders::_1)
        );
    }

    LineNode::~LineNode() {}

    // NOVÉ: Co se stane, když zmáčkneš modré tlačítko
    void LineNode::calibrate_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        is_calibrating_ = true;
        calibration_start_time_ = this->now();

        // Nastavíme extrémně špatné hodnoty, aby je hned první přečtení senzoru přepsalo
        min_l_ = 65535.0f; max_l_ = 0.0f;
        min_r_ = 65535.0f; max_r_ = 0.0f;

        RCLCPP_INFO(this->get_logger(), "KALIBRACE START: Hledam minima a maxima...");
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        uint16_t left = msg->data[0];
        uint16_t right = msg->data[1];

        // LOGIKA KALIBRACE
        if (is_calibrating_) {
            // Kontrola, jestli už uplynuly 3 sekundy
            if ((this->now() - calibration_start_time_).seconds() > 5.0) {
                is_calibrating_ = false; // Konec kalibrace
                RCLCPP_INFO(this->get_logger(), "KALIBRACE HOTOVA! L(min:%.0f max:%.0f) R(min:%.0f max:%.0f)",
                            min_l_, max_l_, min_r_, max_r_);
            } else {
                // Hledání nových minim a maxim
                if (left < min_l_) min_l_ = left;
                if (left > max_l_) max_l_ = left;
                if (right < min_r_) min_r_ = right;
                if (right > max_r_) max_r_ = right;
                return; // Během kalibrace nepočítáme pozici, jen sbíráme data
            }
        }

        // BĚŽNÝ VÝPOČET POZICE (S využitím nových dynamických minim a maxim)

        // Bezpečnostní pojistka proti dělení nulou (kdyby min a max bylo stejné)
        float range_l = (max_l_ - min_l_ > 0.0f) ? (max_l_ - min_l_) : 1.0f;
        float range_r = (max_r_ - min_r_ > 0.0f) ? (max_r_ - min_r_) : 1.0f;

        // Využíváme nakalibrované proměnné místo fixních čísel!
        float l_raw = (left - min_l_) / range_l;
        float r_raw = (right - min_r_) / range_r;

        float l_calibrated = std::clamp(l_raw, 0.0f, 1.0f);
        float r_calibrated = std::clamp(r_raw, 0.0f, 1.0f);

        float current_continuous = algorithms::LineEstimator::estimate_continuous(l_calibrated, r_calibrated);
        //RCLCPP_INFO(this->get_logger(), "KALIBRACE HOTOVA! L %u R %u",
        //                            left, right);
        continuous_.store(current_continuous);
        discrete_.store(algorithms::LineEstimator::estimate_discrete(l_calibrated, r_calibrated));

        std_msgs::msg::Float32 pose_msg;
        pose_msg.data = current_continuous;
        pose_pub_->publish(pose_msg);
    }

} // namespace nodes