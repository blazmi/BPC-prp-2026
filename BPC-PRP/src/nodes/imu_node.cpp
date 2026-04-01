#include "nodes/imu_node.hpp"

namespace nodes {
    ImuNode::ImuNode() : Node("imu_node"), mode(ImuNodeMode::CALIBRATE) {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Node spuštěn. Probíhá kalibrace (nehybejte s robotem!)...");
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(msg->angular_velocity.z);

            // Po nasbírání 200 vzorků (cca 2-4 sekundy) zkalibrujeme
            if (gyro_calibration_samples_.size() >= 200) {
                planar_integrator_.setCalibration(gyro_calibration_samples_);
                mode = ImuNodeMode::INTEGRATE;
                RCLCPP_INFO(this->get_logger(), "Kalibrace hotova. Přepínám na integraci.");
            }
        } else {
            planar_integrator_.update(msg->angular_velocity.z, dt);
            // Zde bys mohl publikovat aktuální yaw na nový topic
        }
    }
}//
// Created by blazmi on 25.03.26.
//