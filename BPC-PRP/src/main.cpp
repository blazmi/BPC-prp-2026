#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"
#include "helper.hpp"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/Line_node.hpp"
#include "loops/line_loop.hpp"
#include "nodes/lidar_node.hpp"
#include "loops/corridor_loop.hpp"
#include "nodes/imu_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Vytvoříš IoNode (ten už JE Node)
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::motorNode>();
    auto encoder_node = std::make_shared<nodes::EncoderNode>();
    //auto line_node = std::make_shared<nodes::LineNode>();
    //auto lineLoop = std::make_shared<nodes::lineLoop>();
    auto lidar_node = std::make_shared<nodes::LidarFilterNode>();
    auto corridor_loop = std::make_shared<nodes::corridorLoop>();
    auto imu_node = std::make_shared<nodes::ImuNode>();
    executor->add_node(io_node);
    executor->add_node(motor_node);
    executor->add_node(encoder_node);
    //executor->add_node(line_node);
    executor->add_node(lidar_node);
    executor->add_node(corridor_loop);
    executor->add_node(imu_node);
//    executor->add_node(lineLoop);

    // Přidáš IoNode do executoru


    //rclcpp::spin(motor_Node);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
