//
// Created by blazmi on 15.03.26.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <functional>
#include <algorithm> // PŘIDÁNO: Nutné pro std::clamp
#include <cmath>     // PŘIDÁNO: Nutné pro std::round

namespace nodes {

    // Konstruktor s opravenou inicializací času (last_time_)
    lineLoop::lineLoop() : rclcpp::Node("lineLoop"),
                           pid_{25.0f, 0.5f, 0.2f},
                           kinematics_{0.034, 0.123, 585},
                           last_time_(this->now()) // TADY JE TA OPRAVA ČASU!
    {

        // Inicializace Publisheru na naše nové vnitřní téma
        cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/line_loop/motor_cmds",
            10
        );

        // Inicializace Subscriberu
        pose_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/line_node/continuous_pose",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->pose_callback(msg); }
        );

        // Inicializace Timeru
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->line_loop_timer_callback(); }
        );
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/enable",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->enable_callback(msg); }
        );
        calibrate_sub_ = this->create_subscription<std_msgs::msg::Empty>(
                "/robot/calibrate",
                10,
                [this](const std_msgs::msg::Empty::SharedPtr msg) { this->calibrate_callback(msg); }
            );
    }
    void lineLoop::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_enabled_ = msg->data;
        if (is_enabled_) {
            RCLCPP_INFO(this->get_logger(), "ROBOT ODSTARTOVAL!");
        } else {
            RCLCPP_INFO(this->get_logger(), "ROBOT ZASTAVEN!");
        }
    }
    void lineLoop::pose_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_line_pose_ = msg->data;
    }

    void lineLoop::line_loop_timer_callback() {
        // 1. Zjištění času a výpočet dt
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) return;

        // ---------------------------------------------------------
        // NOVÉ: 2. Řešení kalibrace (má absolutně nejvyšší prioritu)
        if (is_calibrating_) {
            // Zkontrolujeme, jestli už uplynuly 3 sekundy rotace
            if ((current_time - calibration_start_time_).seconds() > 5.0) {
                is_calibrating_ = false; // Konec kalibrace
                RCLCPP_INFO(this->get_logger(), "Fyzicka rotace dokoncena.");
                // Necháme kód pokračovat dolů, kde ho zachytí buď START nebo STOP
            } else {
                // Jsme v kalibraci: točíme se na místě (Tank turn)
                // 127 je střed. Levé kolo dáme dopředu (180) a pravé dozadu (74)
                std_msgs::msg::UInt8MultiArray calib_msg;
                calib_msg.data = {140, 114};
                cmd_pub_->publish(calib_msg);
                return; // ZÁSADNÍ: Tímto funkci ukončíme a PID se vůbec nepočítá!
            }
        }
        // ---------------------------------------------------------

        // 3. Řešení Start/Stop (Pojistka z minula)
        if (!is_enabled_) {
            std_msgs::msg::UInt8MultiArray stop_msg;
            stop_msg.data = {127, 127};
            cmd_pub_->publish(stop_msg);
            return;
        }
        // 2. PID regulátor vypočítá potřebnou rychlost zatáčení (omega v rad/s)
        float omega = pid_.step(current_line_pose_, dt);

        // 3. Nastavení dopředné rychlosti (v)
        float v_base = 0.05f; // 0.15 m/s je ideální startovní rychlost

        // 4. Výpočet inverzní kinematiky (získáme požadované otáčky kol v rad/s)
        algorithms::RobotSpeed desired_speed{v_base, omega};
        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(desired_speed);

        // 5. Převod fyzikálních otáček (rad/s) na řídicí signál motorů (0 - 255)
        // 127 je střed (stojí). Přičítáme/odečítáme hodnotu vynásobenou naším koeficientem.
        int pwm_l = 127 + std::round(wheel_speeds.l * rad_s_to_pwm_);
        int pwm_r = 127 + std::round(wheel_speeds.r * rad_s_to_pwm_);
        //RCLCPP_INFO(this->get_logger(),"wheels L %d D %d ", pwm_l, pwm_r);
        // 6. Bezpečnostní ořezání (clamp), aby hodnoty nikdy nepřetekly přes povolený limit
        pwm_l = std::clamp(pwm_l, 0, 255);
        pwm_r = std::clamp(pwm_r, 0, 255);

        // 7. Odeslání povelů do motorNode
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = {static_cast<uint8_t>(pwm_l), static_cast<uint8_t>(pwm_r)};
        cmd_pub_->publish(msg);
    }
    void lineLoop::calibrate_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        is_calibrating_ = true;
        calibration_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Fyzicka rotace pro kalibraci START!");
    }
} // namespace nodes