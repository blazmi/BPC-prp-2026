#include "algorithms//LidarFilterResults.hpp"
#include "nodes/lidar_node.hpp" // Tvoje třída z předchozího kroku s logikou filtru


LidarFilterNode::LidarFilterNode() : Node("lidar_filter_node") {

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filtered_distances", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Node spuštěn. Čekám na data a publikuji na 'filtered_distances'...");
}

void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
    algorithms::LidarFilter filter;

    algorithms::LidarFilterResults results = filter.apply_filter(
        msg->ranges,
        msg->angle_min,
        msg->angle_max
    );

    auto pub_msg = std_msgs::msg::Float32MultiArray();

    // [0] = front, [1] = back, [2] = left, [3] = right
    pub_msg.data = {results.front, results.back, results.left, results.right};

    publisher_->publish(pub_msg);

    RCLCPP_INFO(this->get_logger(),
        "Publikováno -> Vpředu: %.2f | Vzadu: %.2f | Vlevo: %.2f | Vpravo: %.2f",
        results.front, results.back, results.left, results.right
    );
}
//
// Created by blazmi on 25.03.26.
//