#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace nodes {
    class LidarFilterNode : public rclcpp::Node {
    public:
        LidarFilterNode();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    };
}