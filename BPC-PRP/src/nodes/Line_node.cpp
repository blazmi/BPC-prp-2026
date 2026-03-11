//
// Created by student on 11.03.26.
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "nodes/Line_node.hpp"

namespace nodes
{

    LineNode::LineNode() : Node("line_node")
    {
        line_sensors_subscriber_ =
            this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                "/bpc_prp_robot/line_sensors",
                10,
                std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
            );
    }

    LineNode::~LineNode()
    {
    }

    float LineNode::get_continuous_line_pose() const
    {
        return continuous;
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() const
    {
        return discrete;
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Line sensor message too small");
            return;
        }

        uint16_t left = static_cast<uint16_t>(msg->data[0]);
        uint16_t right = static_cast<uint16_t>(msg->data[1]);

        auto l_calibrated = (left - 19) / (420 - 19);
        auto r_calibrated = (right - 20) / (510 - 20);

        continuous = estimate_continuous_line_pose(l_calibrated , r_calibrated);
        discrete = estimate_discrete_line_pose(l_calibrated, r_calibrated);

        RCLCPP_INFO(this->get_logger(),
                    "Sensors: L=%u R=%u", left,right );
        (void)continuous;
        (void)discrete;
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value)
    {
        float sum = left_value - right_value;

        if (sum == 0.0f)
            return 0.0f;

        return (right_value - left_value) / sum;
    }

    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm)
    {
        const float threshold = 0.3f;

        bool left_detected = l_norm > threshold;
        bool right_detected = r_norm > threshold;

        if (left_detected && right_detected)
            return DiscreteLinePose::LineBoth;
        if (left_detected)
            return DiscreteLinePose::LineOnLeft;
        if (right_detected)
            return DiscreteLinePose::LineOnRight;

        return DiscreteLinePose::LineNone;
    }

}