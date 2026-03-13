#include "../include/nodes/Line_node.hpp"
#include <algorithm> // Pro std::clamp

namespace nodes {

    LineNode::LineNode() : Node("line_node")
    {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors",
            10,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
        );
    }

    LineNode::~LineNode() {}

    float LineNode::get_continuous_line_pose() const
    {
        return continuous_.load();
    }

    algorithms::DiscreteLinePose LineNode::get_discrete_line_pose() const
    {
        return discrete_.load();
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Line sensor message too small");
            return;
        }

        uint16_t left = msg->data[0];
        uint16_t right = msg->data[1];

        // OPRAVA 1: Použití desetinných čísel (.0f) zabrání uříznutí na 0 nebo 1
        float l_raw = (left - 19.0f) / (420.0f - 19.0f);
        float r_raw = (right - 20.0f) / (510.0f - 20.0f);

        // OPRAVA 2: Clamp zajistí, že číslo nepřeteče mimo povolený rozsah <0.0, 1.0>
        float l_calibrated = std::clamp(l_raw, 0.0f, 1.0f);
        float r_calibrated = std::clamp(r_raw, 0.0f, 1.0f);

        // OPRAVA 3: Použití naší nové samostatné třídy
        continuous_.store(algorithms::LineEstimator::estimate_continuous(l_calibrated, r_calibrated));
        discrete_.store(algorithms::LineEstimator::estimate_discrete(l_calibrated, r_calibrated));

        // Odkomentuj pro debugování:
        // RCLCPP_INFO(this->get_logger(), "Sensors: L=%u R=%u | Cont: %f", left, right, continuous_.load());
    }

} // namespace nodes