#pragma once // Je dobré přidat, aby se hlavička nenačítala vícekrát

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

// ZMĚNA: Přidána zpráva pro pole desetinných čísel z našeho Lidar filtru
#include <std_msgs/msg/float32_multi_array.hpp>

#include "kinematics.hpp"
#include "algorithms/pid.hpp"

namespace nodes {
    class corridorLoop : public rclcpp::Node {
    public:
        corridorLoop();
        // OPRAVENO: Odstraněny závorky navíc
        ~corridorLoop() override = default;

    private:
        // Odstranil jsem proměnné pro kalibraci, s LiDARem ji nepotřebujeme

        // Proměnné a funkce pro povolení jízdy
        bool is_enabled_ = false;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

        // Řízení a kinematika
        algorithms::Pid pid_;
        algorithms::Kinematics kinematics_;
        rclcpp::Time last_time_;

        float rad_s_to_pwm_ = 5.0f; // kolik dilku je potreba aby kolo se tocilo rad/s

        void corridor_loop_timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;

        // Publisher pro motory
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;

        // ZMĚNA: Subscriber pro data z našeho LiDAR filtru
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;

        // ZMĚNA: Callback funkce pro zpracování dat z LiDARu
        void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        // ZMĚNA: Nové proměnné pro řízení průjezdu koridorem
        float current_error_ = 0.0f;   // Odchylka od středu koridoru (levá minus pravá stěna)
        float front_distance_ = 99.0f; // Bezpečnostní vzdálenost (inicializována na vysokou hodnotu)
    };
} // namespace nodes