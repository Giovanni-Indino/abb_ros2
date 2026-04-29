#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <abb_librws/rws_state_machine_interface.h>

class SmartGripperControl : public rclcpp::Node
{
protected:
    std::string ip_robot_;
    int port_robot_rws_;
    double max_force_;
    double max_speed_;

    abb::rws::RWSStateMachineInterface* p_rws_interface_;

    bool setGripperLimits();
    bool sendGripperCommand(const std::string& command);

public:
    SmartGripperControl();
    ~SmartGripperControl();

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grip_in_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grip_out_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;

    void CalibrateSrv(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void GripInSrv(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void GripOutSrv(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};