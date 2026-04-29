#include <smart_gripper_control/smart_gripper_control.h>
#include <chrono>

SmartGripperControl::SmartGripperControl(): rclcpp::Node("sg_control")
{
    declare_parameter<std::string>("robot.ip_robot", "192.168.125.1");
    declare_parameter<int>("robot.robot_port_rws", 80);
    declare_parameter<double>("robot.smart_gripper.max_force", 120.0);
    declare_parameter<double>("robot.smart_gripper.max_speed", 200.0);

    this->get_parameter("robot.ip_robot", ip_robot_);
    this->get_parameter("robot.robot_port_rws", port_robot_rws_);
    this->get_parameter("robot.smart_gripper.max_force", max_force_);
    this->get_parameter("robot.smart_gripper.max_speed", max_speed_);

    // Both grippers
    grip_in_service_ = this->create_service<std_srvs::srv::Trigger>(
        "grip_in", std::bind(&SmartGripperControl::GripInSrv, this,
            std::placeholders::_1, std::placeholders::_2));
    grip_out_service_ = this->create_service<std_srvs::srv::Trigger>(
        "grip_out", std::bind(&SmartGripperControl::GripOutSrv, this,
            std::placeholders::_1, std::placeholders::_2));

    // Calibrate service
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate", std::bind(&SmartGripperControl::CalibrateSrv, this,
            std::placeholders::_1, std::placeholders::_2));

    // Establishing connection with RWS server
    p_rws_interface_ = new abb::rws::RWSStateMachineInterface(
        this->ip_robot_, this->port_robot_rws_);

    RCLCPP_INFO(this->get_logger(), "SmartGripper initialized. IP: %s Port: %d",
        ip_robot_.c_str(), port_robot_rws_);

    // Wait for gripper to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for gripper to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Set max speed and force via I/O signals
    bool r1 = p_rws_interface_->setIOSignal("hand_MaxSpeed_L",  std::to_string((int)max_speed_));
    bool r2 = p_rws_interface_->setIOSignal("hand_MaxSpeed_R",  std::to_string((int)max_speed_));
    bool r3 = p_rws_interface_->setIOSignal("hand_HoldForce_L", std::to_string((int)max_force_));
    bool r4 = p_rws_interface_->setIOSignal("hand_HoldForce_R", std::to_string((int)max_force_));
    RCLCPP_INFO(this->get_logger(), "Set speed/force signals: %d %d %d %d", r1, r2, r3, r4);

    // Set position limits: min=0 (fully closed), max=250 (fully open)
    p_rws_interface_->setIOSignal("hand_MinPosition_L", "0");
    p_rws_interface_->setIOSignal("hand_MinPosition_R", "0");
    p_rws_interface_->setIOSignal("hand_MaxPosition_L", "250");
    p_rws_interface_->setIOSignal("hand_MaxPosition_R", "250");

    // Calibrate
    bool r5 = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "10");
    bool r6 = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "10");
    RCLCPP_INFO(this->get_logger(), "Calibrate signals: %d %d", r5, r6);

    // Wait for calibration to complete
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Calibration complete - ready to receive commands");
}

void SmartGripperControl::CalibrateSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Calibrate called");

    p_rws_interface_->setIOSignal("hand_MinPosition_L", "0");
    p_rws_interface_->setIOSignal("hand_MinPosition_R", "0");
    p_rws_interface_->setIOSignal("hand_MaxPosition_L", "250");
    p_rws_interface_->setIOSignal("hand_MaxPosition_R", "250");

    bool r1 = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "10");
    bool r2 = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "10");
    RCLCPP_INFO(this->get_logger(), "Calibrate results: L=%d R=%d", r1, r2);
    response->success = r1 && r2;
    response->message = response->success ? "Calibrate success" : "Calibrate failed";
}

void SmartGripperControl::GripInSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "GripIn called");

    p_rws_interface_->setIOSignal("hand_MinPosition_L", "0");
    p_rws_interface_->setIOSignal("hand_MinPosition_R", "0");
    p_rws_interface_->setIOSignal("hand_MaxPosition_L", "250");
    p_rws_interface_->setIOSignal("hand_MaxPosition_R", "250");

    bool result_l = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "12");
    bool result_r = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "12");
    RCLCPP_INFO(this->get_logger(), "GripIn results: L=%d R=%d", result_l, result_r);
    response->success = result_l && result_r;
    response->message = response->success ? "Grip in success" : "Grip in failed";
}

void SmartGripperControl::GripOutSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "GripOut called");

    p_rws_interface_->setIOSignal("hand_MinPosition_L", "0");
    p_rws_interface_->setIOSignal("hand_MinPosition_R", "0");
    p_rws_interface_->setIOSignal("hand_MaxPosition_L", "250");
    p_rws_interface_->setIOSignal("hand_MaxPosition_R", "250");

    bool result_l = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "11");
    bool result_r = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "11");
    RCLCPP_INFO(this->get_logger(), "GripOut results: L=%d R=%d", result_l, result_r);
    response->success = result_l && result_r;
    response->message = response->success ? "Grip out success" : "Grip out failed";
}