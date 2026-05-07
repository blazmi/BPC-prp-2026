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
        state_(State::CALIBRATION),
        turn_completed_(false),
        distance_driven_in_intersection_(0.0f)
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

    // --- CAMERA LOGIC: Marker Queue ---
    void MazeLoop::camera_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        int cmd = msg->data;

        // Ignore treasure and unknown commands
        if (cmd != 0 && cmd != 1 && cmd != 2) return;

        // Add to queue safely
        if (marker_queue_.size() < 5 && (marker_queue_.empty() || marker_queue_.back() != cmd)) {
            marker_queue_.push(cmd);
            RCLCPP_INFO(this->get_logger(), "Marker queued: %d (queue size: %zu)",
                        cmd, marker_queue_.size());
        }
    }

    void MazeLoop::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();

        // FIX: Inicializace last_imu_time_ na první zprávu, ne v konstruktoru
        if (last_imu_time_.nanoseconds() == 0) {
            last_imu_time_ = now;
            return;
        }

        double dt = (now - last_imu_time_).seconds();
        last_imu_time_ = now;

        // FIX: Guard na nesmyslné dt (příliš malé nebo příliš velké)
        if (dt <= 0.0 || dt > 0.1) return;

        if (state_ == State::CALIBRATION) {
            calibration_samples_.push_back(msg->angular_velocity.z);
            if (calibration_samples_.size() >= 200) {
                float sum = std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0f);
                gyro_offset_ = sum / calibration_samples_.size();
                current_yaw_ = 0.0f;
                target_yaw_  = 0.0f;
                // FIX: Reset last_imu_time_ po kalibraci, aby první dt po přepnutí
                //      nebylo rovno celé době kalibrace (~2-4s) → obrovský yaw skok
                last_imu_time_ = this->get_clock()->now();
                state_ = State::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "Calibration complete! Offset: %.5f rad/s", gyro_offset_);
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
        left_dist_      = fix_dist(msg->data[2]);
        right_dist_     = fix_dist(msg->data[3]);
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
            case State::INTERSECTION:       handle_intersection(dt);       break;
            case State::TURNING:            handle_turning(dt);            break;
            case State::CALIBRATION:        send_motor_cmd(127, 127);      break;
        }
    }

    // --- STATE 1: CORRIDOR FOLLOWING ---
    void MazeLoop::handle_corridor_following(double dt) {
        const float wall_ok_limit    = 0.40f;
        const float open_space_limit = 0.50f;

        bool in_corridor    = (left_dist_ < wall_ok_limit && right_dist_ < wall_ok_limit);
        bool can_turn_left  = (left_dist_  > open_space_limit);
        bool can_turn_right = (right_dist_ > open_space_limit);
        bool front_blocked  = (front_distance_ < 0.25f);
        bool side_open      = (can_turn_left || can_turn_right);

        // --- SCENARIO A: FRONT BLOCKED (We must turn immediately) ---
        if (front_blocked) {

            // 1. Dead End
            if (!side_open) {
                target_yaw_ += M_PI;
                RCLCPP_INFO(this->get_logger(), "Dead end! Turning 180.");
            }
            // 2. T-Intersection (both sides open → pop queue)
            else if (can_turn_left && can_turn_right) {
                int current_marker = -1;
                if (!marker_queue_.empty()) {
                    current_marker = marker_queue_.front();
                    marker_queue_.pop();
                    RCLCPP_INFO(this->get_logger(), "T-Intersection: Using marker %d", current_marker);
                } else {
                    RCLCPP_WARN(this->get_logger(), "T-Intersection: Queue empty! Defaulting RIGHT.");
                }

                if (current_marker == 1) {
                    target_yaw_ += (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "Turning LEFT.");
                } else if (current_marker == 2) {
                    target_yaw_ -= (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "Turning RIGHT.");
                } else {
                    target_yaw_ -= (M_PI / 2.0f);
                    RCLCPP_WARN(this->get_logger(), "Invalid marker for T-Intersection! Defaulting RIGHT.");
                }
            }
            // 3. Simple Corner Left
            else if (can_turn_left) {
                target_yaw_ += (M_PI / 2.0f);
                RCLCPP_INFO(this->get_logger(), "Corner: Turning LEFT.");
            }
            // 4. Simple Corner Right
            else if (can_turn_right) {
                target_yaw_ -= (M_PI / 2.0f);
                RCLCPP_INFO(this->get_logger(), "Corner: Turning RIGHT.");
            }

            state_ = State::TURNING;
            turn_completed_ = false;
            distance_driven_in_intersection_ = 0.0f;
            return;
        }

        // --- SCENARIO B: FRONT CLEAR + SIDE OPEN (True Intersection) ---
        if ((front_distance_ >= 0.65f) && side_open && (state_ != State::INTERSECTION && state_ != State::TURNING)) {
            state_ = State::INTERSECTION;
            distance_driven_in_intersection_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), "Intersection detected! Moving to center...");
            return;
        }

        // --- PID Control in Corridor ---
        float omega  = 0.0f;
        float v_base = 0.16f;

        if (in_corridor) {
            float Kp_lidar = 1.2f;
            float Ki_lidar = 0.4f;

            if (std::abs(current_error_) > 0.01f) {
                lidar_integral_ += current_error_ * static_cast<float>(dt);
            }
            lidar_integral_ = std::clamp(lidar_integral_, -0.5f, 0.5f);

            float lidar_correction = (current_error_ * Kp_lidar) + (lidar_integral_ * Ki_lidar);
            float desired_yaw = target_yaw_ + std::clamp(lidar_correction, -0.4f, 0.4f);
            float yaw_error   = desired_yaw - current_yaw_;

            while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 4.0f;
        } else {
            float yaw_error = target_yaw_ - current_yaw_;
            while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            omega = yaw_error * 3.0f;
            v_base = 0.10f;
            lidar_integral_ = 0.0f;
        }

        publish_kinematics(v_base, std::clamp(omega, -1.2f, 1.2f));
    }

    // --- STATE 2: INTERSECTION (Move to center) ---
    void MazeLoop::handle_intersection(double dt) {
        float yaw_error = target_yaw_ - current_yaw_;
        float v_base = 0.10f;
        if (left_dist_ < 0.12f)       yaw_error -= 20.0f * M_PI / 180.0f;
        else if (right_dist_ < 0.12f) yaw_error += 20.0f * M_PI / 180.0f;

        while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;



        distance_driven_in_intersection_ += v_base * static_cast<float>(dt);

        float omega = std::clamp(yaw_error * 6.0f, -0.5f, 0.5f);
        publish_kinematics(v_base, omega);

        if (distance_driven_in_intersection_ >= 0.08f) {

            int current_marker = -1;
            if (!marker_queue_.empty()) {
                current_marker = marker_queue_.front();
                marker_queue_.pop();
                RCLCPP_INFO(this->get_logger(), "Using marker from queue: %d (remaining: %zu)",
                            current_marker, marker_queue_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "No marker in queue! Defaulting STRAIGHT.");
                current_marker = 2;
            }

            bool can_turn_left  = (left_dist_  > 0.50f);
            bool can_turn_right = (right_dist_ > 0.50f);

            distance_driven_in_intersection_ = 0.0f;

            if (current_marker == 1 && can_turn_left) {
                target_yaw_ += (M_PI / 2.0f);
                state_ = State::TURNING;
                turn_completed_ = false;
                RCLCPP_INFO(this->get_logger(), "Center reached: Turning LEFT.");
            }

            else if (current_marker == 0) {
                target_yaw_ += 0;
                state_ = State::TURNING;
                turn_completed_ = false;
                RCLCPP_INFO(this->get_logger(), "Center reached: Continuing STRAIGHT.");
            }
            else if (can_turn_right && current_marker == 2) {
                target_yaw_ -= (M_PI / 2.0f);
                state_ = State::TURNING;
                turn_completed_ = false;
                RCLCPP_INFO(this->get_logger(), "Center reached: Turning RIGHT.");
            }
        }
    }

    // --- STATE 3: TURNING IN PLACE + EXIT ---
    void MazeLoop::handle_turning(double dt) {
        float yaw_error = target_yaw_ - current_yaw_;

        // Korekce při příliš blízké stěně během otáčení
        if (left_dist_ < 0.1f)       yaw_error -= 20.0f * M_PI / 180.0f;
        else if (right_dist_ < 0.1f) yaw_error += 20.0f * M_PI / 180.0f;

        while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        // Phase 1: Turn in place
        if (!turn_completed_) {
            // FIX: Zvětšený deadband 0.05f (místo 0.01f)
            //      IMU drift způsoboval, že robot threshold 0.01f nikdy nedosáhl
            //      a otáčel se dál → přetočení
            if (std::abs(yaw_error) < 0.05f) {
                turn_completed_ = true;
                distance_driven_in_intersection_ = 0.0f;
                lidar_integral_ = 0.0f;
                RCLCPP_INFO(this->get_logger(), "Turn done, exiting intersection...");
            } else {
                float omega = std::clamp(yaw_error * 3.5f, -1.3f, 1.3f);

                // FIX: Minimální omega jen pokud je robot daleko od cíle (> 0.15 rad ~8.6°)
                //      Původní pevné minimum 0.5f způsobovalo překmit při dobrůžení k cíli
                if (std::abs(yaw_error) > 0.15f && std::abs(omega) < 0.3f) {
                    omega = (omega > 0) ? 0.3f : -0.3f;
                }

                publish_kinematics(0.0f, omega);
            }
            return;
        }

        // Phase 2: Drive straight to exit intersection
        float v_base = 0.1f;
        distance_driven_in_intersection_ += v_base * static_cast<float>(dt);

        float omega = std::clamp(yaw_error * 5.0f, -0.5f, 0.5f);
        publish_kinematics(v_base, omega);

        if (distance_driven_in_intersection_ >= 0.24f) {
            turn_completed_ = false;
            distance_driven_in_intersection_ = 0.0f;
            state_ = State::CORRIDOR_FOLLOWING;
            RCLCPP_INFO(this->get_logger(), "Exited intersection, entering corridor.");
        }
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