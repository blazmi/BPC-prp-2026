#include "nodes/lidar_node.hpp"
#include "algorithms/LidarFilterResults.hpp" // Opraveno dvojité lomítko v cestě

namespace nodes {

    // Doporučuji specifikovat plný název třídy rclcpp::Node
    LidarFilterNode::LidarFilterNode() : rclcpp::Node("lidar_filter_node") {

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filtered_distances", 10);

        // OPRAVA QoS: Pro spojení se senzory musíme použít SensorDataQoS
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar",
            10,
            std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Lidar Node spuštěn. Čekám na data a publikuji na 'filtered_distances'...");
    }

    void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
        // OCHRANA: Pokud senzor pošle poškozená (prázdná) data, zamezíme pádu nodu
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Přijata prázdná data z LIDARu (ranges je prázdné)!");
            return;
        }

        algorithms::LidarFilter filter;

        //VYUŽITÍ INCREMENTU: Předáme increment ze zprávy místo maximálního úhlu
        algorithms::LidarFilterResults results = filter.apply_filter(
           msg->ranges,
            msg->angle_min,
           msg->angle_increment
        );


        auto pub_msg = std_msgs::msg::Float32MultiArray();

         //[0] = front, [1] = back, [2] = left, [3] = right
        pub_msg.data = {results.front, results.back, results.left, results.right};

        publisher_->publish(pub_msg);

        // Vypisujeme už jen reálné vzdálenosti (původní matoucí výpis jsem smazal)
        RCLCPP_INFO(this->get_logger(),
            "Publikováno -> Vpředu: %.2f | Vzadu: %.2f | Vlevo: %.2f | Vpravo: %.2f",
            results.front, results.back, results.left, results.right
       );
    }
}