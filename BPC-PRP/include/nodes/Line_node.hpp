//
// Created by student on 11.03.26.
//

#ifndef PRP_PROJECT_LINE_NODE_HPP
#define PRP_PROJECT_LINE_NODE_HPP
#include <std_msgs/msg/u_int16_multi_array.hpp>
// Public API sketch; adapt to your project
namespace nodes {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineNode : public rclcpp::Node {
    public:
        LineNode();
        ~LineNode();

        // Relative pose to line [m]
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

    private:
        DiscreteLinePose discrete;
        float continuous;

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };
}


#endif //PRP_PROJECT_LINE_NODE_HPP