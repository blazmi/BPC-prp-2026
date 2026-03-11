#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"
#include "helper.hpp"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/Line_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Vytvoříš IoNode (ten už JE Node)
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::motorNode>();
    //auto encoder_node = std::make_shared<nodes::EncoderNode>();
    auto line_node = std::make_shared<nodes::LineNode>();
    executor->add_node(io_node);
    executor->add_node(motor_node);
    //executor->add_node(encoder_node);
    executor->add_node(line_node);

    while (1) {
        motor_node->set_motor_vel(100,100);
        //executor->spin();
    }

    // Přidáš IoNode do executoru


    //rclcpp::spin(motor_Node);

        executor->spin();

    rclcpp::shutdown();
    return 0;
}
