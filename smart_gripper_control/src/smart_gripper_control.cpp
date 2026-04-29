#include <smart_gripper_control/smart_gripper_control.h>
#include <chrono>

SmartGripperControl::SmartGripperControl()
: rclcpp::Node("sg_control"),
  p_rws_interface_(nullptr)
{
    declare_parameter<std::string>("robot.ip_robot", "192.168.125.1");
    declare_parameter<int>("robot.robot_port_rws", 80);
    declare_parameter<double>("robot.smart_gripper.max_force", 120.0);
    declare_parameter<double>("robot.smart_gripper.max_speed", 200.0);

    this->get_parameter("robot.ip_robot", ip_robot_);
    this->get_parameter("robot.robot_port_rws", port_robot_rws_);
    this->get_parameter("robot.smart_gripper.max_force", max_force_);
    this->get_parameter("robot.smart_gripper.max_speed", max_speed_);

    grip_in_service_ = this->create_service<std_srvs::srv::Trigger>(
        "grip_in",
        std::bind(
            &SmartGripperControl::GripInSrv,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    grip_out_service_ = this->create_service<std_srvs::srv::Trigger>(
        "grip_out",
        std::bind(
            &SmartGripperControl::GripOutSrv,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate",
        std::bind(
            &SmartGripperControl::CalibrateSrv,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    p_rws_interface_ = new abb::rws::RWSStateMachineInterface(
        this->ip_robot_,
        this->port_robot_rws_
    );

    RCLCPP_INFO(
        this->get_logger(),
        "SmartGripper initialized. IP: %s Port: %d",
        ip_robot_.c_str(),
        port_robot_rws_
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for gripper to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    bool r1 = p_rws_interface_->setIOSignal(
        "hand_MaxSpeed_L",
        std::to_string(static_cast<int>(max_speed_))
    );

    bool r2 = p_rws_interface_->setIOSignal(
        "hand_MaxSpeed_R",
        std::to_string(static_cast<int>(max_speed_))
    );

    bool r3 = p_rws_interface_->setIOSignal(
        "hand_HoldForce_L",
        std::to_string(static_cast<int>(max_force_))
    );

    bool r4 = p_rws_interface_->setIOSignal(
        "hand_HoldForce_R",
        std::to_string(static_cast<int>(max_force_))
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Set speed/force signals: %d %d %d %d",
        r1,
        r2,
        r3,
        r4
    );

    bool limits_ok = setGripperLimits();
    RCLCPP_INFO(this->get_logger(), "Set limits result: %d", limits_ok);

    bool calib_ok = sendGripperCommand("10");
    RCLCPP_INFO(this->get_logger(), "Initial calibration command result: %d", calib_ok);

    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(
        this->get_logger(),
        "Calibration command sent - ready to receive commands"
    );
}

SmartGripperControl::~SmartGripperControl()
{
    if (p_rws_interface_ != nullptr)
    {
        delete p_rws_interface_;
        p_rws_interface_ = nullptr;
    }
}

bool SmartGripperControl::setGripperLimits()
{
    bool r1 = p_rws_interface_->setIOSignal("hand_MinPosition_L", "0");
    bool r2 = p_rws_interface_->setIOSignal("hand_MinPosition_R", "0");
    bool r3 = p_rws_interface_->setIOSignal("hand_MaxPosition_L", "250");
    bool r4 = p_rws_interface_->setIOSignal("hand_MaxPosition_R", "250");

    RCLCPP_INFO(
        this->get_logger(),
        "Set position limits: minL=%d minR=%d maxL=%d maxR=%d",
        r1,
        r2,
        r3,
        r4
    );

    return r1 && r2 && r3 && r4;
}

bool SmartGripperControl::sendGripperCommand(const std::string& command)
{
    if (p_rws_interface_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "RWS interface is null");
        return false;
    }

    bool limits_ok = setGripperLimits();

    bool reset_1_l = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "0");
    bool reset_1_r = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "0");

    RCLCPP_INFO(
        this->get_logger(),
        "Reset before command: L=%d R=%d",
        reset_1_l,
        reset_1_r
    );

    rclcpp::sleep_for(std::chrono::milliseconds(200));

    bool cmd_l = p_rws_interface_->setIOSignal("hand_CmdGripper_L", command);
    bool cmd_r = p_rws_interface_->setIOSignal("hand_CmdGripper_R", command);

    RCLCPP_INFO(
        this->get_logger(),
        "Command %s sent: L=%d R=%d",
        command.c_str(),
        cmd_l,
        cmd_r
    );

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    bool reset_2_l = p_rws_interface_->setIOSignal("hand_CmdGripper_L", "0");
    bool reset_2_r = p_rws_interface_->setIOSignal("hand_CmdGripper_R", "0");

    RCLCPP_INFO(
        this->get_logger(),
        "Reset after command: L=%d R=%d",
        reset_2_l,
        reset_2_r
    );

    return limits_ok &&
           reset_1_l && reset_1_r &&
           cmd_l && cmd_r &&
           reset_2_l && reset_2_r;
}

void SmartGripperControl::CalibrateSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    RCLCPP_INFO(this->get_logger(), "Calibrate called");

    bool ok = sendGripperCommand("10");

    response->success = ok;
    response->message = ok ? "Calibrate success" : "Calibrate failed";
}

void SmartGripperControl::GripInSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    RCLCPP_INFO(this->get_logger(), "GripIn called");

    bool ok = sendGripperCommand("12");

    response->success = ok;
    response->message = ok ? "Grip in success" : "Grip in failed";
}

void SmartGripperControl::GripOutSrv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    RCLCPP_INFO(this->get_logger(), "GripOut called");

    bool ok = sendGripperCommand("11");

    response->success = ok;
    response->message = ok ? "Grip out success" : "Grip out failed";
}