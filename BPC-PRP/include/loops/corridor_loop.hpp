#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp> // NOVÉ: Pro data z MPU6050

#include <vector>   // NOVÉ: Pro kalibrační vzorky
#include <numeric>

#include "kinematics.hpp"
#include "algorithms/pid.hpp"

namespace nodes {

    // NOVÉ: Definice stavů robota
    enum class State {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        TURNING
    };

    class corridorLoop : public rclcpp::Node {
    public:
        corridorLoop();
        ~corridorLoop() override = default;

    private:
        // Callbacky
        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); // NOVÉ
        void corridor_loop_timer_callback();

        // Pomocné metody pro logiku stavů (aby nebyl timer_callback příliš dlouhý)
        void handle_corridor_following(double dt);
        void handle_turning();
        void publish_kinematics(float v, float omega);
        void send_motor_cmd(int l, int r);

        // Stav a povolení jízdy
        State state_; // Aktuální stav (začíná CALIBRATION)
        bool is_enabled_ = false;

        // Řízení a kinematika
        algorithms::Pid pid_;
        algorithms::Kinematics kinematics_;
        rclcpp::Time last_time_;
        float rad_s_to_pwm_ = 5.0f;

        // Komunikace (Pub/Sub)
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // NOVÉ

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Data z LiDARu
        float current_error_ = 0.0f;
        float front_distance_ = 99.0f;
        float left_dist_ = 99.0f;  // NOVÉ
        float right_dist_ = 99.0f; // NOVÉ

        // Data z IMU a Yaw integrace
        float current_yaw_ = 0.0f;     // Integrovaný úhel (radians)
        float target_yaw_ = 0.0f;      // Cílový úhel pro otočku
        float gyro_offset_ = 0.0f;     // Drift vypočtený při kalibraci
        rclcpp::Time last_imu_time_;   // Pro výpočet dt v imu_callbacku
        std::vector<float> calibration_samples_; // Vzorky pro průměrování driftu
    };

} // namespace nodes