#ifndef PRP_PROJECT_LINE_NODE_HPP
#define PRP_PROJECT_LINE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp> // Zde jsem opravil podtržítko na standardní ROS2 formát
#include <std_msgs/msg/float32.hpp>            // NOVÉ: Zpráva pro odeslání floatu
#include <atomic>
#include "../algorithms/line_estimator.hpp"
#include <std_msgs/msg/empty.hpp> // PŘIDÁNO: Knihovna pro prázdnou zprávu
namespace nodes {

    class LineNode : public rclcpp::Node {
    public:
        LineNode();
        ~LineNode();

        float get_continuous_line_pose() const;
        algorithms::DiscreteLinePose get_discrete_line_pose() const;

    private:
        bool is_calibrating_ = false;
        rclcpp::Time calibration_start_time_;

        // Výchozí hodnoty (kdybys zapomněl kalibrovat, ať to aspoň nějak jede)
        float min_l_ = 19.0f, max_l_ = 200.0f;
        float min_r_ = 20.0f, max_r_ = 500.0f;

        // NOVÉ: Subscriber pro kalibraci
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_sub_;
        void calibrate_callback(const std_msgs::msg::Empty::SharedPtr msg);



        std::atomic<algorithms::DiscreteLinePose> discrete_{algorithms::DiscreteLinePose::LineNone};
        std::atomic<float> continuous_{0.0f};

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        // NOVÉ: Přidáváme Publisher pro pozici čáry
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pose_pub_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    };

} // namespace nodes
#endif