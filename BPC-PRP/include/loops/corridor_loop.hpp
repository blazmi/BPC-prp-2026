#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>
#include <numeric>

#include "kinematics.hpp"
#include "algorithms/pid.hpp"

namespace nodes {

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
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void corridor_loop_timer_callback();

        // Pomocné metody pro stavový stroj
        void handle_corridor_following(double dt);
        void handle_turning();
        void publish_kinematics(float v, float omega);
        void send_motor_cmd(int l, int r);

        // Stav a řízení
        float reference_yaw_ = 0.0f; // Pamatuje si směr chodby, než zmizela stěna
        State state_;
        bool is_enabled_ = false;

        algorithms::Pid pid_;
        algorithms::Kinematics kinematics_;
        rclcpp::Time last_time_;
        float rad_s_to_pwm_ = 5.0f;

        // Komunikace
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // LiDAR data
        float current_error_ = 0.0f;
        float front_distance_ = 99.0f;
        float left_dist_ = 99.0f;
        float right_dist_ = 99.0f;

        // IMU data
        float current_yaw_ = 0.0f;
        float target_yaw_ = 0.0f;
        float gyro_offset_ = 0.0f;
        rclcpp::Time last_imu_time_;
        std::vector<float> calibration_samples_;
    };

} // namespace nodes