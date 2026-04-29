#include <rclcpp/rclcpp.hpp>
#include <smart_gripper_control/smart_gripper_control.h>

int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a shared pointer to your node
    auto node = std::make_shared<SmartGripperControl>();

    // Spin the node (process callbacks)
    rclcpp::spin(node);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();

    return 0;
}