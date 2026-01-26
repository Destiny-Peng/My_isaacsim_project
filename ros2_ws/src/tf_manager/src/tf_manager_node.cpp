#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_manager_node");
    RCLCPP_INFO(node->get_logger(), "tf_manager_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
