#include "loops/corridor_loop.hpp"
#include <chrono>
#include <functional>
#include <algorithm>
#include <cmath>

namespace nodes {

    corridorLoop::corridorLoop() : rclcpp::Node("corridorLoop"),
        pid_{1.0f, 0.0f, 0.1f},
        kinematics_{0.034, 0.123, 585},
        last_time_(this->get_clock()->now()),
        last_imu_time_(this->get_clock()->now()),
        state_(State::CALIBRATION)
    {
        cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/corridor_loop/motor_cmds", 10);
        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "filtered_distances", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->lidar_callback(msg); });
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imu_callback(msg); });
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/enable", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { this->enable_callback(msg); });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->corridor_loop_timer_callback(); });
    }

    void corridorLoop::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();
        if (last_imu_time_.nanoseconds() == 0) { last_imu_time_ = now; return; }
        double dt = (now - last_imu_time_).seconds();
        last_imu_time_ = now;
        if (dt <= 0 || dt > 0.1) return;

        if (state_ == State::CALIBRATION) {
            calibration_samples_.push_back(msg->angular_velocity.z);
            if (calibration_samples_.size() >= 200) {
                float sum = std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0f);
                gyro_offset_ = sum / calibration_samples_.size();
                current_yaw_ = 0.0f;
                target_yaw_ = 0.0f;
                state_ = State::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "Kalibrace hotova!");
            }
        } else {
            float corrected_gyro = msg->angular_velocity.z - gyro_offset_;
            current_yaw_ += corrected_gyro * static_cast<float>(dt);
        }
    }

    void corridorLoop::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;

        // OŠETŘENÍ SLEPÉ ZÓNY (15cm): Pokud senzor vrací 0, znamená to, že je zeď hrozně blízko
        auto fix_dist = [](float d) {
            if (d <= 0.001f) return 0.05f; // Přepíšeme 0 na 5cm, aby regulace zabrala
            return d;
        };

        front_distance_ = fix_dist(msg->data[0]);
        left_dist_ = fix_dist(msg->data[2]);
        right_dist_ = fix_dist(msg->data[3]);

        // Výpočet chyby pro LiDAR - pouze v koridoru (stěny < 0.6m)
        if (left_dist_ < 0.6f && right_dist_ < 0.6f) {
            current_error_ = left_dist_ - right_dist_;
        }
    }

    void corridorLoop::corridor_loop_timer_callback() {
        if (!is_enabled_) { send_motor_cmd(127, 127); return; }
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0) return;

        switch (state_) {
            case State::CORRIDOR_FOLLOWING: handle_corridor_following(dt); break;
            case State::TURNING: handle_turning(); break;
            case State::CALIBRATION: send_motor_cmd(127, 127); break;
        }
    }

    void corridorLoop::handle_corridor_following(double dt) {
        // 1. ROZHODNUTÍ: JSME V KORIDORU?
        // Hranice, kdy stěnu ještě bereme jako vodítko
        const float wall_ok_limit = 0.50f;
        bool in_corridor = (left_dist_ < wall_ok_limit && right_dist_ < wall_ok_limit);

        // 2. DETEKCE ZATÁČKY (předek < 0.35m)
        if (front_distance_ < 0.25f) {
            if (left_dist_ > wall_ok_limit) {
                target_yaw_ += (M_PI / 2.0f); // Doleva
                state_ = State::TURNING;
                return;
            } else if (right_dist_ > wall_ok_limit) {
                target_yaw_ -= (M_PI / 2.0f); // Doprava
                state_ = State::TURNING;
                return;
            }
        }

        float omega = 0.0f;
        float v_base = 0.10f;

        if (in_corridor) {
            // JÍZDA V KORIDORU - PI regulace
            float Kp_lidar = 1.2f; // Zvýšeno, aby se odlepil od zdi
            float Ki_lidar = 0.3f;

            if (std::abs(current_error_) > 0.01f) {
                lidar_integral_ += current_error_ * static_cast<float>(dt);
            }
            lidar_integral_ = std::clamp(lidar_integral_, -0.5f, 0.5f);

            float lidar_correction = (current_error_ * Kp_lidar) + (lidar_integral_ * Ki_lidar);

            // Cíl je udržet se rovně (target_yaw) s drobnou korekcí od LiDARu
            float desired_yaw = target_yaw_ + std::clamp(lidar_correction, -0.4f, 0.4f);
            float yaw_error = desired_yaw - current_yaw_;

            // Normalizace
            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 4.0f; // Silná vnitřní smyčka IMU
        } else {
            // HEADING LOCK (Křižovatka/Ztráta stěny)
            // Držíme striktně směr bez LiDARu
            float yaw_error = target_yaw_ - current_yaw_;
            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 3.0f;
            v_base = 0.08f; // Zpomalíme pro jistotu
            lidar_integral_ = 0.0f; // Resetujeme paměť chyb
        }

        publish_kinematics(v_base, std::clamp(omega, -1.2f, 1.2f));
    }

    void corridorLoop::handle_turning() {
        float yaw_error = target_yaw_ - current_yaw_;
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        if (std::abs(yaw_error) < 0.05f) {
            state_ = State::CORRIDOR_FOLLOWING;
            lidar_integral_ = 0.0f;
            return;
        }

        float Kp_turn = 3.5f;
        float omega = yaw_error * Kp_turn;
        float max_omega = 1.3f;
        omega = std::clamp(omega, -max_omega, max_omega);
        if (std::abs(omega) < 0.5f) omega = (omega > 0) ? 0.5f : -0.5f;

        publish_kinematics(0.0f, omega);
    }

    void corridorLoop::publish_kinematics(float v, float omega) {
        algorithms::RobotSpeed desired_speed{v, omega};
        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(desired_speed);
        int pwm_l = 127 + std::round(wheel_speeds.l * rad_s_to_pwm_);
        int pwm_r = 127 + std::round(wheel_speeds.r * rad_s_to_pwm_);
        send_motor_cmd(std::clamp(pwm_l, 0, 255), std::clamp(pwm_r, 0, 255));
    }

    void corridorLoop::send_motor_cmd(int l, int r) {
        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = {static_cast<uint8_t>(l), static_cast<uint8_t>(r)};
        cmd_pub_->publish(out_msg);
    }

    void corridorLoop::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_enabled_ = msg->data;
        if (!is_enabled_) send_motor_cmd(127, 127);
    }

} // namespace nodes