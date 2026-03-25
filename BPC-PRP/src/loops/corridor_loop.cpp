//
// Created by blazmi on 15.03.26.
//
#include "loops/corridor_loop.hpp"

#include <chrono>
#include <functional>
#include <algorithm> // Nutné pro std::clamp
#include <cmath>     // Nutné pro std::round

namespace nodes {

    // Konstruktor
    corridorLoop::corridorLoop() : rclcpp::Node("corridorLoop"), pid_{25.0f, 0.5f, 0.2f},
         kinematics_{0.034, 0.123, 585},
         last_time_(this->now())
    {
        // Inicializace Publisheru pro motory
        cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/corridor_loop/motor_cmds",
            10
        );

        // ZMĚNA: Posloucháme pole vzdáleností z našeho Lidar filtru
        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "filtered_distances", // Topic z předchozího kódu
            10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->lidar_callback(msg); }
        );

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/enable",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->enable_callback(msg); }
        );

        // Inicializace Timeru
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->corridor_loop_timer_callback(); }
        );
    }

    void corridorLoop::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_enabled_ = msg->data;
        if (is_enabled_) {
            RCLCPP_INFO(this->get_logger(), "ROBOT ODSTARTOVAL DO KORIDORU!");
        } else {
            RCLCPP_INFO(this->get_logger(), "ROBOT ZASTAVEN!");
        }
    }

    // ZMĚNA: Nový callback pro zpracování LiDAR dat
    void corridorLoop::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Kontrola, zda máme všechny 4 hodnoty (vpředu, vzadu, vlevo, vpravo)
        if (msg->data.size() < 4) return;

        float front = msg->data[0];
        float left = msg->data[2];
        float right = msg->data[3];

        // Uložíme si vzdálenost před námi pro případné nouzové zastavení
        front_distance_ = front;

        // VÝPOČET CHYBY PRO PID
        // Pokud jedna ze stěn zmizí (vrací nekonečno), chyba je dočasně 0 (jedeme rovně)
        if (std::isinf(left) || std::isinf(right)) {
            current_error_ = 0.0f;
        } else {
            // Rozdíl vzdáleností.
            // Kladná chyba = levá stěna je dál (jsme moc vpravo) -> zatáčíme doleva
            // Záporná chyba = pravá stěna je dál (jsme moc vlevo) -> zatáčíme doprava
            current_error_ = left - right;
        }
    }

    void corridorLoop::corridor_loop_timer_callback() {
        // 1. Zjištění času a výpočet dt
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) return;

        // 2. Řešení Start/Stop a bezpečnostní brzda (pokud jsme moc blízko zdi vepředu)
        // Zastavíme, pokud nejsme zapnutí, nebo pokud je překážka blíž než 0.25 metru
        if (!is_enabled_ || front_distance_ < 0.25f) {
            std_msgs::msg::UInt8MultiArray stop_msg;
            stop_msg.data = {127, 127}; // 127 = stojíme
            cmd_pub_->publish(stop_msg);

            if (front_distance_ < 0.25f && is_enabled_) {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Prekazka vepredu! Zastavuji.");
            }
            return;
        }

        // 3. PID regulátor vypočítá potřebnou rychlost zatáčení (omega v rad/s)
        // Snažíme se srazit current_error_ na 0
        float omega = pid_.step(current_error_, dt);

        // 4. Nastavení dopředné rychlosti (v)
        float v_base = 0.15f;

        // 5. Výpočet inverzní kinematiky (získáme požadované otáčky kol v rad/s)
        algorithms::RobotSpeed desired_speed{v_base, omega};
        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(desired_speed);

        // 6. Převod fyzikálních otáček (rad/s) na řídicí signál motorů (0 - 255)
        int pwm_l = 127 + std::round(wheel_speeds.l * rad_s_to_pwm_);
        int pwm_r = 127 + std::round(wheel_speeds.r * rad_s_to_pwm_);

        // 7. Bezpečnostní ořezání (clamp)
        pwm_l = std::clamp(pwm_l, 0, 255);
        pwm_r = std::clamp(pwm_r, 0, 255);

        // 8. Odeslání povelů
        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = {static_cast<uint8_t>(pwm_l), static_cast<uint8_t>(pwm_r)};
        cmd_pub_->publish(out_msg);
    }

} // namespace nodes