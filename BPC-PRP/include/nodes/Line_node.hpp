#ifndef PRP_PROJECT_LINE_NODE_HPP
#define PRP_PROJECT_LINE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <atomic>
#include "../algorithms/line_estimator.hpp" // Přidáme náš estimátor

namespace nodes {

    class LineNode : public rclcpp::Node {
    public:
        LineNode();
        ~LineNode();

        float get_continuous_line_pose() const;
        algorithms::DiscreteLinePose get_discrete_line_pose() const;

    private:
        // Atomic zajistí, že čtení a zápis ze dvou různých vláken nespadne
        std::atomic<algorithms::DiscreteLinePose> discrete_{algorithms::DiscreteLinePose::LineNone};
        std::atomic<float> continuous_{0.0f};

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    };

} // namespace nodes
#endif