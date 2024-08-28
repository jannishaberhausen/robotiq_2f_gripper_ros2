#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <robotiq_2f_gripper_interfaces/default_driver.hpp>
#include <robotiq_2f_gripper_interfaces/default_serial.hpp>

#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>

using namespace robotiq_2f_gripper_hardware;
using namespace std::placeholders;
using robotiq_2f_gripper_interfaces::DefaultDriver;
using robotiq_2f_gripper_interfaces::DefaultSerial;


GripperNode::GripperNode() : Node("robotiq_2f_gripper_node") {
    declare_parameter<std::string>("serial_port");
    serial_port_ = get_parameter("serial_port").as_string();
    RCLCPP_INFO(get_logger(), "Using serial port: %s", serial_port_.c_str());

    declare_parameter<int>("baudrate");
    baudrate_ = get_parameter("baudrate").as_int();
    RCLCPP_INFO(get_logger(), "Using baudrate: %d", baudrate_);

    declare_parameter<double>("timeout");
    timeout_ = get_parameter("timeout").as_double();
    RCLCPP_INFO(get_logger(), "Using timeout: %f", timeout_);

    declare_parameter<int>("action_timeout");
    action_timeout_ = get_parameter("action_timeout").as_int();
    RCLCPP_INFO(get_logger(), "Using action timeout: %d", action_timeout_);

    declare_parameter<int>("slave_address");
    slave_address_ = get_parameter("slave_address").as_int();
    RCLCPP_INFO(get_logger(), "Using slave address: %d", slave_address_);

    declare_parameter<bool>("fake_hardware");
    fake_hardware_ = get_parameter("fake_hardware").as_bool();
    RCLCPP_INFO(get_logger(), "Using fake hardware: %s", fake_hardware_ ? "true" : "false");

    if (!fake_hardware_) {
        auto serial = std::make_unique<DefaultSerial>();
        serial->set_port(serial_port_);
        serial->set_baudrate(baudrate_);
        serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout_)));

        driver_ = std::make_unique<DefaultDriver>(std::move(serial));
        driver_->set_slave_address(slave_address_);

        const bool connected = driver_->connect();
        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "The gripper is not connected");
            return;
        }
        RCLCPP_INFO(get_logger(), "The gripper is connected.");

        if (!(driver_->is_gripper_active()))
        {
            driver_->deactivate();
            driver_->activate();
        }
        RCLCPP_INFO(get_logger(), "The gripper is activated.");
    }

    action_server_ = rclcpp_action::create_server<SetPosition>(
        this, "robotiq_2f_gripper_action",
        std::bind(&GripperNode::handle_move_goal, this, _1, _2),
        std::bind(&GripperNode::handle_cancel_move, this, _1),
        std::bind(&GripperNode::handle_move_accepted, this, _1));

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("robotiq_2f_gripper/joint_states", 1);
    timer_1_ = create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&GripperNode::update_joint_state_callback, this));

    gripper_state_publisher_ = create_publisher<std_msgs::msg::Int32>("robotiq_2f_gripper/gripper_state", 1);
    timer_2_ = create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&GripperNode::update_gripper_state_callback, this));

    RCLCPP_INFO(get_logger(), "Gripper node initialized");
}

GripperNode::~GripperNode() {
    if (!fake_hardware_) {
        driver_->deactivate();
        driver_->disconnect();
    }
}

rclcpp_action::GoalResponse GripperNode::handle_move_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const SetPosition::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request with %f meters", goal->target_position);

    if (running_) {
        RCLCPP_WARN(get_logger(), "Discarding new goal request, previous goal still running");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!fake_hardware_) {
        if (!driver_->is_gripper_active()) {
            RCLCPP_ERROR(get_logger(), "Gripper is not activated");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // Check if the goal is valid
    // if (goal->target_position < 0 || goal->target_position > 0.140001) {
    //     RCLCPP_ERROR(get_logger(), "Invalid goal request, target position (in meters [m]) must be between 0 (fully closed) and 0.14 (fully open)");
    //     return rclcpp_action::GoalResponse::REJECT;
    // }
    // if (goal->target_speed < 0 || goal->target_speed > 1) {
    //     RCLCPP_ERROR(get_logger(), "Invalid goal request, target speed must be between 0 and 1");
    //     return rclcpp_action::GoalResponse::REJECT;
    // }
    // if (goal->target_force < 0 || goal->target_force > 1) {
    //     RCLCPP_ERROR(get_logger(), "Invalid goal request, target force must be between 0 and 1");
    //     return rclcpp_action::GoalResponse::REJECT;
    // }

    running_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperNode::handle_cancel_move(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    running_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperNode::handle_move_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Accepted goal");
    std::thread{std::bind(&GripperNode::execute, this, _1), goal_handle}.detach();
}

void GripperNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetPosition::Feedback>();
    auto result = std::make_shared<SetPosition::Result>();

    if (fake_hardware_) {
        gripper_position_ = goal->target_position;
        gripper_speed_ = 0;
        gripper_force_ = goal->target_force;

        RCLCPP_INFO(get_logger(), "[Fake Hardware] Gripper movement completed.");
        result->success = true;
        goal_handle->succeed(result);
        running_ = false;
        return;
    }

    try {
        feedback->feedback = "Setting force";
        goal_handle->publish_feedback(feedback);
        driver_->set_force(convertToGripperSystem(goal->target_force));

        feedback->feedback = "Setting speed";
        goal_handle->publish_feedback(feedback);
        driver_->set_speed(convertToGripperSystem(goal->target_speed));

        feedback->feedback = "Setting position";
        goal_handle->publish_feedback(feedback);
        // convertToGripperSystemPosition
        driver_->set_gripper_position(convertToGripperSystemPosition(goal->target_position));

        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(action_timeout_);
        while (std::chrono::steady_clock::now() - start_time < timeout)
        {
            if (!driver_->gripper_is_moving())
            {
                RCLCPP_INFO(get_logger(), "Gripper movement completed.");
                result->success = true;
                goal_handle->succeed(result);
                running_ = false;
                return;
            }
            
            // provide feedback about ongoing movement
            feedback->feedback = "Gripper is moving";
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(get_logger(), "Error executing goal: %s", e.what());
        result->success = false;
        goal_handle->abort(result);
        running_ = false;
        return;
    }
    RCLCPP_INFO(get_logger(), "Gripper movement timed out.");
    result->success = false;
    goal_handle->abort(result);
    running_ = false;
}

void GripperNode::update_joint_state_callback() {
    if (running_) {
        return;
    }

    double curr_gripper_position;
    if (!fake_hardware_) {
        curr_gripper_position = convertToMillimeters(static_cast<int>(driver_->get_gripper_position()));
    }
    else {
        curr_gripper_position = gripper_position_;
    }

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = now();
    message.name = {"gripper_distance"};    
    message.position = {curr_gripper_position};
    joint_state_publisher_->publish(message);
}

void GripperNode::update_gripper_state_callback() {
    if (running_) {
        return;
    }

    auto message = std_msgs::msg::Int32();
    message.data = 0;
    gripper_state_publisher_->publish(message);
}

uint8_t GripperNode::decimalToHex(int value)
{
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

int GripperNode::convertToGripperSystemPosition(double position)
{
    if (position >= 0.015) {
        double a = -1399.78;
        double b = -1328.11;
        double c = 218.384;
        return decimalToHex(static_cast<int>(a * pow(position, 2) + b * position + c));
    } 
    else if (position >= 0.001)
    {
        double a = 18315;
        double b = -1849.82;
        double c = 223.626;
        return decimalToHex(static_cast<int>(a * pow(position, 2) + b * position + c));
    } 
    else
    {
        return decimalToHex(255);
    }
}

// inverse of convertToGripperSystemPosition
double GripperNode::convertToMillimeters(int value)
{
    // strong approximation (assuming linear relationship)
    return static_cast<double>((value - 255) * -1 * 0.14 / 255);
}

int GripperNode::convertToGripperSystem(double value)
{
    return decimalToHex(static_cast<int>(value * 255));
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}