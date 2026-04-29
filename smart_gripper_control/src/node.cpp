#include <rclcpp/rclcpp.hpp>
#include <smart_gripper_control/smart_gripper_control.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SmartGripperControl>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}