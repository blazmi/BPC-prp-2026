#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp> // NOVÉ: Pro kameru
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>
#include <numeric>

// Uprav cesty k vlastním hlavičkám, pokud jsou jinak
#include "kinematics.hpp"
#include "algorithms/pid.hpp"

namespace nodes {

    enum class State {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        INTERSECTION, // NOVÝ STAV
        TURNING
    };

    class MazeLoop : public rclcpp::Node {
    public:
        MazeLoop();
        ~MazeLoop() override = default;

    private:
        // Callbacky
        // ... (mezi privátní metody)
        void handle_intersection(double dt); // NOVÁ FUNKCE

        // ... (mezi privátní proměnné k in_intersection_)
        float distance_driven_in_intersection_ = 0.0f; // Počítadlo pro těch 18 cm



        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void camera_callback(const std_msgs::msg::UInt8::SharedPtr msg); // NOVÉ
        void corridor_loop_timer_callback();

        // Pomocné metody pro stavový stroj
        void handle_corridor_following(double dt);
        void handle_turning();
        void publish_kinematics(float v, float omega);
        void send_motor_cmd(int l, int r);

        // Stav a řízení
        float reference_yaw_ = 0.0f;
        State state_;
        bool is_enabled_ = false;

        algorithms::Pid pid_;
        algorithms::Kinematics kinematics_;
        rclcpp::Time last_time_;
        float rad_s_to_pwm_ = 5.0f;

        // Paměť pro značky (Kamera)
        // -1 = žádná značka v paměti, 0 = rovně, 1 = doleva, 2 = doprava
        int saved_marker_ = -1;

        // Komunikace
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_sub_; // NOVÉ
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
        float lidar_integral_ = 0.0f;
        // ... (původní kód) ...

        // Paměť pro značky (Kamera)
        //int saved_marker_ = -1;

        // NOVÉ: Proměnné pro řešení problému s dvojitou značkou
        int cached_marker_ = -1;     // Záložní paměť pro značku chycenou v křižovatce
        bool in_intersection_ = false; // Příznak, že se robot fyzicky nachází v křižovatce

        // ... (zbytek) ...
    };

} // namespace nodes