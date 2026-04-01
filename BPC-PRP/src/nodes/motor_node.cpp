//
// Created by student on 04.03.26.
//
/* 585 pulsu na otacku, muze za to madar*/
#include "nodes/motor_node.hpp"
namespace nodes {
    motorNode::motorNode() : rclcpp::Node("motor") {
        motor_vel_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds",
            rclcpp::QoS(10)
        );

        // Inicializace Subscriberu
        cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/corridor_loop/motor_cmds",
            10,
            [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) { this->cmd_callback(msg); }
        );

    }

    // Tady zpracujeme přijatou zprávu od lineLoopu
    void motorNode::cmd_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        // Bezpečnostní kontrola, jestli nám opravdu přišly dvě hodnoty
        if (msg->data.size() >= 2) {
            uint8_t vel_l = msg->data[0];
            uint8_t vel_r = msg->data[1];

            // Zavoláme tvoji stávající metodu, která to pošle robotovi!
            set_motor_vel(vel_l, vel_r);
        }
    }

    void motorNode::set_motor_vel(uint8_t vel_l, uint8_t vel_r )
    {
        std_msgs::msg::UInt8MultiArray msg;

        msg.data = { vel_l, vel_r};

        motor_vel_pub_->publish(msg);

       /* RCLCPP_INFO(this->get_logger(),
                    "motor speed: [%u, %u]",
                    vel_l, vel_r );*/
    }



    EncoderNode::EncoderNode()
    : Node("encoder_node")
    {
        sub_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/bpc_prp_robot/encoders",
            10,
            std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1)
        );
    }

    void EncoderNode::encoder_callback(
        const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Encoder message too small!");
            return;
        }

        int left_encoder  = msg->data[0];
        int right_encoder = msg->data[1];

     //   RCLCPP_INFO(this->get_logger(),
     //               "Encoders: L=%u R=%u",
     //               left_encoder, right_encoder);
    }

    // ...
}