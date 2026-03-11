//
// Created by student on 04.03.26.
//

#ifndef PRP_PROJECT_MOTOR_NODE_HPP
#define PRP_PROJECT_MOTOR_NODE_HPP


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {
    class motorNode : public rclcpp::Node {
    public:
        // Constructor
        motorNode();

        // Destructor (default)
        ~motorNode() override = default;

        // Function to retrieve the last pressed button value


    private:
        void timer_callback();
        void set_motor_vel(uint8_t vel_l = 150, uint8_t vel_r = 150);

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };


    class EncoderNode : public rclcpp::Node
    {
    public:
        EncoderNode();

    private:
        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_;
    };

}



#endif //PRP_PROJECT_MOTOR_NODE_HPP