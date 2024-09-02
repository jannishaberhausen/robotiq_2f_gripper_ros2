#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>

#include <robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp>

namespace robotiq_2f_gripper_hardware {

using SetPosition = robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper;
using robotiq_2f_gripper_interfaces::DefaultDriver;
using robotiq_2f_gripper_interfaces::DefaultSerial;

class GripperNode : public rclcpp::Node {
    public:
        GripperNode();
        ~GripperNode();
    
    private:
        std::string serial_port_;
        int baudrate_;
        double timeout_;
        int action_timeout_;
        int slave_address_;
        bool fake_hardware_;
        std::unique_ptr<DefaultDriver> driver_;

        int gripper_position_;
        int gripper_speed_;
        int gripper_force_;

        rclcpp_action::Server<SetPosition>::SharedPtr action_server_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_state_publisher_;

        std::atomic<bool> running_{false};
        sensor_msgs::msg::JointState joint_state_;
        rclcpp::TimerBase::SharedPtr timer_1_, timer_2_;

        rclcpp_action::GoalResponse handle_move_goal(
            const rclcpp_action::GoalUUID& /*uuid*/,
            std::shared_ptr<const SetPosition::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_move(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> /*goal_handle*/);
        void handle_move_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle);
        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle);
        void update_joint_state_callback();
        void update_gripper_state_callback();

        uint8_t decimalToHex(int value);
        int convertToGripperSystemPosition(double position);
        double convertToMillimeters(int value);
        int convertToGripperSystem(double value);

        void move_to_goal_(
            const std::shared_ptr<const SetPosition::Goal> goal,
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle,
            std::shared_ptr<SetPosition::Feedback> feedback,
            std::shared_ptr<SetPosition::Result> result);
        void move_to_contact_(
            const std::shared_ptr<const SetPosition::Goal> goal,
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle,
            std::shared_ptr<SetPosition::Feedback> feedback,
            std::shared_ptr<SetPosition::Result> result);
};

}  // namespace robotiq_2f_gripper_hardware