#include "loops/corridor_loop.hpp"
#include <chrono>
#include <functional>
#include <algorithm>
#include <cmath>

namespace nodes {

    corridorLoop::corridorLoop() : rclcpp::Node("corridorLoop"),
        pid_{1.0f, 0.1f, 0.05f},
       kinematics_{0.034, 0.123, 585},
       // TADY JE ZMĚNA: Používáme hodiny nodu místo obecného now()
       last_time_(this->get_clock()->now()),
       last_imu_time_(this->get_clock()->now()),
       state_(State::CALIBRATION)
    {
        // Publishery
        cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/corridor_loop/motor_cmds", 10);

        // Subscribery
        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "filtered_distances", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->lidar_callback(msg); });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imu_callback(msg); });

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/enable", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { this->enable_callback(msg); });

        // Timer (100 Hz pro hladké řízení)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->corridor_loop_timer_callback(); });

        RCLCPP_INFO(this->get_logger(), "Corridor Loop inicializován. Čekám na kalibraci IMU...");
    }

    void corridorLoop::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();

        // První průběh: last_imu_time_ může být z jiné doby, tak ho jen zinicializujeme
        if (last_imu_time_.nanoseconds() == 0) {
            last_imu_time_ = now;
            return;
        }

        double dt = (now - last_imu_time_).seconds();
        last_imu_time_ = now;

        if (dt <= 0 || dt > 0.1) return; // Ochrana proti nesmyslným skokům v čase

        if (state_ == State::CALIBRATION) {
            // Sběr vzorků pro offset (drift)
            calibration_samples_.push_back(msg->angular_velocity.z);
            if (calibration_samples_.size() >= 200) {
                float sum = std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0f);
                gyro_offset_ = sum / calibration_samples_.size();
                state_ = State::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "Kalibrace hotova! Offset: %.4f. Startuji CORRIDOR_FOLLOWING.", gyro_offset_);
            }
        } else {
            // Integrace YAW (úhlu natočení)
            float corrected_gyro = msg->angular_velocity.z - gyro_offset_;
            current_yaw_ += corrected_gyro * static_cast<float>(dt);
        }
    }

    void corridorLoop::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        front_distance_ = msg->data[0];
        left_dist_ = msg->data[2];
        right_dist_ = msg->data[3];

        // Výpočet chyby pro PID (vzdálenost od středu)
        if (std::isinf(left_dist_) || std::isinf(right_dist_)) {
            current_error_ = 0.0f;
        } else {
            current_error_ = left_dist_ - right_dist_;
        }
    }

    void corridorLoop::corridor_loop_timer_callback() {
        if (!is_enabled_) {
            send_motor_cmd(127, 127);
            return;
        }

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        switch (state_) {
            case State::CALIBRATION:
                send_motor_cmd(127, 127); // Stát při kalibraci
                break;

            case State::CORRIDOR_FOLLOWING:
                handle_corridor_following(dt);
                break;

            case State::TURNING:
                handle_turning();
                break;
        }
    }

    void corridorLoop::handle_corridor_following(double dt) {
        // 1. Detekce rohu zůstává podobná
        if (front_distance_ < 0.35f) {
            if (left_dist_ > 0.8f) {
                base_yaw_ += M_PI / 2.0f; // Nový základní směr je o 90° vlevo
                state_ = State::TURNING;
                return;
            } else if (right_dist_ > 0.8f) {
                base_yaw_ -= M_PI / 2.0f; // Nový základní směr je o 90° vpravo
                state_ = State::TURNING;
                return;
            }
        }

        // 2. KASKÁDOVÁ REGULACE
        // Výpočet korekce z LiDARu (převádíme metr na radiány)
        // K_lidar_to_yaw určuje, jak moc agresivně se chceme vracet na střed
        float K_lidar_to_yaw = 0.5f;
        float lidar_correction = current_error_ * K_lidar_to_yaw;

        // Limitujeme korekci, aby robot nezačal dělat prudké manévry (max 15 stupňů)
        lidar_correction = std::clamp(lidar_correction, -0.26f, 0.26f);

        // Cílový úhel pro IMU = základní směr chodby + korekce pro vystředění
        float target_yaw = base_yaw_ + lidar_correction;

        // 3. PID regulátor nyní hlídá ÚHEL (yaw), ne vzdálenost
        float yaw_error = target_yaw - current_yaw_;

        // Normalizace chyby úhlu
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        float omega = pid_.step(yaw_error, dt);
        float v_base = 0.12f; // Konstantní dopředná rychlost

        publish_kinematics(v_base, omega);
    }

    void corridorLoop::handle_turning() {
        // Cílem je dosáhnout base_yaw_ (který jsme změnili při detekci rohu)
        float yaw_error = base_yaw_ - current_yaw_;

        // Normalizace
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        if (std::abs(yaw_error) < 0.05f) {
            state_ = State::CORRIDOR_FOLLOWING;
            RCLCPP_INFO(this->get_logger(), "Zatáčka vybrána, pokračuji rovně.");
            return;
        }

        // Plynulé otáčení: čím blíž jsme cíli, tím pomaleji se točíme
        float max_omega = 0.8f; // Maximální rychlost otáčení (rad/s)
        float Kp_turn = 2.0f;
        float omega = yaw_error * Kp_turn;

        // Omezení maximální rychlosti, aby to nebylo "násilné"
        omega = std::clamp(omega, -max_omega, max_omega);

        // Minimální rychlost, aby se robot neodstavil kvůli tření (deadband)
        if (std::abs(omega) < 0.2f) omega = (omega > 0) ? 0.2f : -0.2f;

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

        if (is_enabled_) {
            RCLCPP_INFO(this->get_logger(), "ROBOT AKTIVOVÁN - Startuji logiku koridoru.");
        } else {
            RCLCPP_INFO(this->get_logger(), "ROBOT DEAKTIVOVÁN - Zastavuji motory.");
            // Při vypnutí rovnou pošleme stopku, nečekáme na timer
            send_motor_cmd(127, 127);
        }
    }

} // namespace nodes