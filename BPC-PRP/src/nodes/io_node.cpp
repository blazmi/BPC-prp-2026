#include "nodes/io_node.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp> // PŘIDÁNO

namespace nodes {
    IoNode::IoNode() : rclcpp::Node("io_node")
    {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons",
            10,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/rgb_leds",
            rclcpp::QoS(10)
        );

        enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/robot/enable",
            10
        );

        // NOVÉ: Inicializace publisheru pro kalibraci
        calibrate_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/robot/calibrate",
            10
        );
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        button_pressed_ = msg->data;

        std_msgs::msg::Bool enable_msg;
        std_msgs::msg::Empty calibrate_msg; // Prázdná zpráva pro kalibraci

        if (button_pressed_ == 0) {
            set_led(100, 0, 0, 100);   // červená = STOP
            enable_msg.data = false;
            enable_pub_->publish(enable_msg);
            RCLCPP_INFO(this->get_logger(), "Odesílám povel: STOP");
        }
        else if (button_pressed_ == 1) {
            set_led(0, 100, 0, 100);   // zelená = START
            enable_msg.data = true;
            enable_pub_->publish(enable_msg);
            RCLCPP_INFO(this->get_logger(), "Odesílám povel: START");
        }
        else if (button_pressed_ == 2) {
            // Použil jsem raději konkrétní číslo tlačítka, aby to bylo přesné
            set_led(0, 0, 100, 100);   // modrá = KALIBRACE
            calibrate_pub_->publish(calibrate_msg); // Pošleme prázdnou zprávu
            RCLCPP_INFO(this->get_logger(), "Odesílám povel: KALIBRUJ");
        }
    }

    void IoNode::set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = { r, g, b, a };
        led_publisher_->publish(msg);
    }
} // namespace nodes#include "nodes/io_node.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp> // PŘIDÁNO
