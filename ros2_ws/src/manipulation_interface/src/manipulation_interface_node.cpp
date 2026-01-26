#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("manipulation_interface_node");
    RCLCPP_INFO(node->get_logger(), "manipulation_interface_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
