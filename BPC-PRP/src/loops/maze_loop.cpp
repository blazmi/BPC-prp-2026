#include "loops/maze_loop.hpp"
#include <chrono>
#include <functional>
#include <algorithm>
#include <cmath>

namespace nodes {

    MazeLoop::MazeLoop() : rclcpp::Node("mazeLoop"),
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

        camera_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/camera/command", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) { this->camera_callback(msg); });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->corridor_loop_timer_callback(); });
    }

    void MazeLoop::camera_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        int cmd = msg->data;
        if (cmd == 0 || cmd == 1 || cmd == 2) {
            if (in_intersection_) {
                if (cached_marker_ != cmd) {
                    cached_marker_ = cmd;
                    RCLCPP_INFO(this->get_logger(), "V KRIZOVATCE! Cachuji znacku na priste: %d", cached_marker_);
                }
            }
            else {
                if (saved_marker_ != cmd) {
                    saved_marker_ = cmd;
                    RCLCPP_INFO(this->get_logger(), "Nova znacka v pameti: %d", saved_marker_);
                }
            }
        }
    }

    void MazeLoop::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
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

    void MazeLoop::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        auto fix_dist = [](float d) { return (d <= 0.001f) ? 0.05f : d; };
        front_distance_ = fix_dist(msg->data[0]);
        left_dist_ = fix_dist(msg->data[2]);
        right_dist_ = fix_dist(msg->data[3]);

        if (left_dist_ < 0.6f && right_dist_ < 0.6f) {
            current_error_ = left_dist_ - right_dist_;
        }
    }

    void MazeLoop::corridor_loop_timer_callback() {
        if (!is_enabled_) { send_motor_cmd(127, 127); return; }
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0) return;

        switch (state_) {
            case State::CORRIDOR_FOLLOWING: handle_corridor_following(dt); break;
            case State::INTERSECTION:       handle_intersection(dt); break;
            case State::TURNING:            handle_turning(); break;
            case State::CALIBRATION:        send_motor_cmd(127, 127); break;
        }
    }

    // --- STAV 1: JÍZDA V KORIDORU ---
    void MazeLoop::handle_corridor_following(double dt) {
        const float wall_ok_limit = 0.30f;
        const float open_space_limit = 0.40f;

        bool in_corridor = (left_dist_ < wall_ok_limit && right_dist_ < wall_ok_limit);
        bool can_turn_left = (left_dist_ > open_space_limit);
        bool can_turn_right = (right_dist_ > open_space_limit);

        bool front_blocked = (front_distance_ < 0.25f);
        bool side_open = (can_turn_left || can_turn_right);

        // 1. VÝJEZD Z KŘIŽOVATKY / ODBOČKY (Zdi jsou zpět)
        if (in_intersection_ && !side_open) {
            in_intersection_ = false;
            if (cached_marker_ != -1) {
                saved_marker_ = cached_marker_;
                cached_marker_ = -1;
                RCLCPP_INFO(this->get_logger(), "Zaviram cache. Prenasim znacku do pameti: %d", saved_marker_);
            }
        }

        // --- SCÉNÁŘ A: ZEĎ VEPŘEDU (Zatáčka, T-křižovatka nebo Slepá ulička) ---
        // NEMUSÍME POPOJÍŽDĚT! Jsme na místě, rovnou točíme.
        if (front_blocked) {
            // Slepá ulička
            if (!side_open) {
                target_yaw_ += M_PI;
                state_ = State::TURNING;
                saved_marker_ = -1;
                RCLCPP_INFO(this->get_logger(), "Slepa ulicka! Tocim se o 180.");
                return;
            }

            // T-Křižovatka (Volno vlevo i vpravo)
            if (can_turn_left && can_turn_right) {
                if (saved_marker_ == 1) {
                    target_yaw_ += (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "T-krizovatka: Zatacim DOLEVA (dle pameti).");
                } else if (saved_marker_ == 2) {
                    target_yaw_ -= (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "T-krizovatka: Zatacim DOPRAVA (dle pameti).");
                } else {
                    // Pojistka, kdyby nebyla značka nebo byla nesmyslná (např. 0 = rovně do zdi)
                    target_yaw_ -= (M_PI / 2.0f);
                    RCLCPP_WARN(this->get_logger(), "T-krizovatka: Chybi znacka! Volim DOPRAVA.");
                }
            }
            // Obyčejná zatáčka doleva
            else if (can_turn_left) {
                target_yaw_ += (M_PI / 2.0f);
                RCLCPP_INFO(this->get_logger(), "Zatacka: Tocim DOLEVA.");
            }
            // Obyčejná zatáčka doprava
            else if (can_turn_right) {
                target_yaw_ -= (M_PI / 2.0f);
                RCLCPP_INFO(this->get_logger(), "Zatacka: Tocim DOPRAVA.");
            }

            state_ = State::TURNING;
            saved_marker_ = -1; // Značka spotřebována
            return;
        }

        // --- SCÉNÁŘ B: VEPŘEDU VOLNO A CHYBÍ BOČNÍ ZEĎ (Popojíždíme) ---
        if ((front_distance_ > 0.5f) && side_open && !in_intersection_) {
            in_intersection_ = true;
            state_ = State::INTERSECTION;
            distance_driven_in_intersection_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), "Odbacka detekovana! Popojizdim do stredu... front %f  left %f  right %f",front_distance_,left_dist_, right_dist_);
            return;
        }

        // --- Samotná jízda v koridoru (PID) ---
        float omega = 0.0f;
        float v_base = 0.10f;

        if (in_corridor) {
            float Kp_lidar = 1.2f;
            float Ki_lidar = 0.3f;

            if (std::abs(current_error_) > 0.01f) {
                lidar_integral_ += current_error_ * static_cast<float>(dt);
            }
            lidar_integral_ = std::clamp(lidar_integral_, -0.5f, 0.5f);

            float lidar_correction = (current_error_ * Kp_lidar) + (lidar_integral_ * Ki_lidar);
            float desired_yaw = target_yaw_ + std::clamp(lidar_correction, -0.4f, 0.4f);
            float yaw_error = desired_yaw - current_yaw_;

            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 4.0f;
        } else {
            float yaw_error = target_yaw_ - current_yaw_;
            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 3.0f;
            v_base = 0.08f;
            lidar_integral_ = 0.0f;
        }

        publish_kinematics(v_base, std::clamp(omega, -1.2f, 1.2f));
    }

    // --- STAV 2: KŘIŽOVATKA (Popojetí 18 cm do středu s volným předkem) ---
    void MazeLoop::handle_intersection(double dt) {
        float v_base = 0.08f;
        distance_driven_in_intersection_ += v_base * static_cast<float>(dt);

        bool can_turn_left = (left_dist_ > 0.40f);
        bool can_turn_right = (right_dist_ > 0.40f);

        // Dojeli jsme 18 cm -> Učíníme rozhodnutí
        if (distance_driven_in_intersection_ >= 0.1f) {

            if (saved_marker_ == 1 && can_turn_left) {
                target_yaw_ += (M_PI / 2.0f);
                state_ = State::TURNING;
                saved_marker_ = -1;
                RCLCPP_INFO(this->get_logger(), "Stred dosazen: Zatacim DOLEVA podle znacky.");
            }
            else if (saved_marker_ == 2 && can_turn_right) {
                target_yaw_ -= (M_PI / 2.0f);
                state_ = State::TURNING;
                saved_marker_ = -1;
                RCLCPP_INFO(this->get_logger(), "Stred dosazen: Zatacim DOPRAVA podle znacky.");
            }
            else {
                // Jízda rovně (buď značka 0, nebo chybí značka k odbočení)
                if (saved_marker_ == 0) saved_marker_ = -1;
                state_ = State::CORRIDOR_FOLLOWING;
                saved_marker_ = cached_marker_;
                cached_marker_ = -1;
                RCLCPP_INFO(this->get_logger(), "Stred dosazen: Pokracuji ROVNE.");
            }
            return;
        }

        // Dokud nedojedeme 18 cm, držíme rovný směr podle IMU
        float yaw_error = target_yaw_ - current_yaw_;
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        float omega = yaw_error * 3.0f;
        publish_kinematics(v_base, std::clamp(omega, -1.2f, 1.2f));
    }

    // --- STAV 3: ZATÁČENÍ NA MÍSTĚ ---
    void MazeLoop::handle_turning() {
        float yaw_error = target_yaw_ - current_yaw_;
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        if (std::abs(yaw_error) < 0.05f) {
            state_ = State::CORRIDOR_FOLLOWING;
            lidar_integral_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), "Otoceni dokonceno, zpet do koridoru.");
            return;
        }

        float Kp_turn = 3.5f;
        float omega = yaw_error * Kp_turn;
        float max_omega = 1.3f;
        omega = std::clamp(omega, -max_omega, max_omega);

        if (std::abs(omega) < 0.5f) omega = (omega > 0) ? 0.5f : -0.5f;

        publish_kinematics(0.0f, omega);
    }

    void MazeLoop::publish_kinematics(float v, float omega) {
        algorithms::RobotSpeed desired_speed{v, omega};
        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(desired_speed);
        int pwm_l = 127 + std::round(wheel_speeds.l * rad_s_to_pwm_);
        int pwm_r = 127 + std::round(wheel_speeds.r * rad_s_to_pwm_);
        send_motor_cmd(std::clamp(pwm_l, 0, 255), std::clamp(pwm_r, 0, 255));
    }

    void MazeLoop::send_motor_cmd(int l, int r) {
        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = {static_cast<uint8_t>(l), static_cast<uint8_t>(r)};
        cmd_pub_->publish(out_msg);
    }

    void MazeLoop::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_enabled_ = msg->data;
        if (!is_enabled_) send_motor_cmd(127, 127);
    }

} // namespace nodes