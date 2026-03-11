#include "nodes/io_node.hpp"
namespace nodes {
    IoNode::IoNode()
   : rclcpp::Node("topic")   // Initialize base Node with node name
   {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons",          // Topic name
            10,                      // QoS (queue size)
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/bpc_prp_robot/rgb_leds",
        rclcpp::QoS(10)
    );
   }


    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        button_pressed_ = msg->data;

        if (button_pressed_ == 0) {
            set_led(100, 0, 0, 100);   // červená
        }
        else if (button_pressed_ == 1) {
            set_led(0, 100, 0, 100);   // zelená
        }
        else {
            set_led(0, 0, 100, 100);   // modrá
        }
    }

    void IoNode::set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        std_msgs::msg::UInt8MultiArray msg;

        msg.data = { r, g, b, a };

        led_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "LED RGBA: [%d, %d, %d, %d]",
                    r, g, b, a);
    }

 // ...
}


