#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp> // Knihovna pro naši zprávu
#include <std_msgs/msg/bool.hpp>
#include "kinematics.hpp"
#include "algorithms/pid.hpp"
#include <std_msgs/msg/empty.hpp> // PŘIDÁNO: Knihovna pro prázdnou zprávu

namespace nodes {
    class lineLoop : public rclcpp::Node {
    public:
        lineLoop();
        ~lineLoop() override = default;



    private:
        bool is_calibrating_ = false;
        rclcpp::Time calibration_start_time_;

        // NOVÉ: Subscriber pro kalibraci
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_sub_;
        void calibrate_callback(const std_msgs::msg::Empty::SharedPtr msg);

        bool is_enabled_ = false;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
        algorithms::Pid pid_;
        algorithms::Kinematics kinematics_;
        rclcpp::Time last_time_;
        float rad_s_to_pwm_ = 5.0f; // kolik dilku je potreba aby kolo se tocilo rad/s

        void line_loop_timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;

        // NOVÉ: Přidáváme Publisher
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;
        // Funkce, která se zavolá, když přijde nová pozice čáry
        void pose_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pose_sub_;


        // Sem si budeme ukládat poslední známou pozici, aby ji timer mohl použít
        float current_line_pose_ = 0.0f;
    };
}